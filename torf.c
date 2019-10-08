/**
* Get UDP MPEGTS and send to IT9500 modulator
*
* @author: Karim Davoodi
* @email:  KarimDavoodi@gmail.com
* @date:   July 2019
*
*/
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <linux/hdreg.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <stdint.h>
#include <semaphore.h>
struct iq {
	uint32_t freq;
	int16_t  amp;
	int16_t  phi;
};
int frequency;
int	capture_mode = 0;
int p_psk = 2;		//  QAM64
int p_coderate = 4; //  7/8
int p_interval = 0; //  1/32
int debug = 0;
int sockfd;
socklen_t addr_len;
struct sockaddr_in servaddr,cliaddr;

#include "api.h"
#include "crc.h"
//#define	KD_LINE KLOG("file:%s func:%s line:%d\n",__FILE__,__func__,__LINE__);
//#define	KD_LINE KLOG("line:%d\n",__LINE__);

#define ERROR(x...)												\
	do {														\
	   KLOG(x);										\
	} while (0)

#define PERROR(x...)											\
	do {														\
	   KLOG(x);										\
	} while (0)


typedef struct {
	char *name;
	int value;
} Param;

typedef struct {
	Dword Frequency;
	Word Bandwidth;
} ModulatorParam;

typedef struct {
	uint32_t count;
	uint32_t ulErrCount;
	uint32_t ulLostCount;
	uint16_t pid;
	uint8_t  sequence1;
	uint8_t  sequence2;
	uint8_t  dup_flag;
} PIDINFO, *PPIDINFO;

#define LIST_SIZE(x) sizeof(x)/sizeof(Param)
#define TX_DATA_LEN 65424

int gTransferRate = 0;
uint32_t gTransferInterval = 0;
int gRateControl = 0; 
Dword g_ChannelCapacity = 0;
Byte NullPacket[188]={0x47,0x1f,0xff,0x1c,0x00,0x00};
MODULATION_PARAM g_ChannelModulation_Setting;

//***************** IQTable Constant *****************//
#define IQTABLE_NROW 65536
#define Default_IQtable_Path "./bin/IQ_table.bin"

//Test Periodical Custom Packets Insertion, (for SI/PSI table insertion)

//Sample SDT
Byte CustomPacket_1[188]={
	0x47,0x40,0x11,0x10,0x00,0x42,0xF0,0x36,0x00,0x99,0xC1,0x00,0x00,0xFF,0x1A,0xFF,
	0x03,0xE8,0xFC,0x80,0x25,0x48,0x23,0x01,0x10,0x05,0x49,0x54,0x45,0x20,0X20,0X20,
	0X20,0X20,0X20,0X20,0X20,0X20,0X20,0X20,0X20,0x10,0x05,0x49,0x54,0x45,0x20,0x43,
	0x68,0x61,0x6E,0x6E,0x65,0x6C,0x20,0x31,0x20,0x20,0xFF,0xFF,0xFF,0xFF //LAST 4 BYTE=CRC32
};

//Sample NIT
Byte CustomPacket_2[188]={
	0x47,0x40,0x10,0x10,0x00,0x40,0xf0,0x38,0xff,0xaf,0xc1,0x00,0x00,0xf0,0x0d,0x40,/*0x00000000*/
	0x0b,0x05,0x49,0x54,0x45,0x20,0x4f,0x66,0x66,0x69,0x63,0x65,0xf0,0x1e,0x00,0x99,/*0x00000010*/
	0xff,0x1a,0xf0,0x18,0x83,0x04,0x03,0xe8,0xfc,0x5f,0x5a,0x0b,0x02,0xeb,0xae,0x40,/*0x00000020*/
	0x1f,0x42,0x52,0xff,0xff,0xff,0xff,0x41,0x03,0x03,0xe8,0x01,0x1a,0xe6,0x2c,0x3f,/*0x00000030*/
};

Byte CustomPacket_3[188]={0x47,0x10,0x03,0x1c,0x00,0x00};
Byte CustomPacket_4[188]={0x47,0x10,0x04,0x1c,0x00,0x00};
Byte CustomPacket_5[188]={0x47,0x10,0x05,0x1c,0x00,0x00};


int kbhit(void)  
{  
	struct termios oldt, newt;  
	int ch;  
	int oldf;  
	tcgetattr(STDIN_FILENO, &oldt);  
	newt = oldt;  
	newt.c_lflag &= ~(ICANON | ECHO);  
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
	ch = getchar();  
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
	fcntl(STDIN_FILENO, F_SETFL, oldf);  
	if(ch != EOF) {  
		ungetc(ch, stdin);  
		return 1;  
	}  
	return 0;  
}  

intmax_t GetFileSize(const char* filePath)
{
	struct stat statbuf;
	
	if(stat(filePath, &statbuf) == -1)
		return -1;
	return (intmax_t) statbuf.st_size;
}

Dword SetIQCalibrationTable(Byte DevNo) {
	FILE	*pfile = NULL;
    Word	error = ERR_NO_ERROR;
	char	inputFile[128] = {0};
	Byte*	ptrIQtable;
	Word	IQtableSize;

	error = scanf ("%s", inputFile);	
	
	pfile = fopen(inputFile, "rb");
	if(pfile == NULL) {
		pfile = fopen(Default_IQtable_Path,"rb");
		if(pfile == NULL) {
			   error = ERR_OPEN_FILE_FAIL;
			   KLOG("Load default IQtable fail with status 0x%08x !", error);
			   return error;
		} else	{
			IQtableSize = GetFileSize(Default_IQtable_Path);
		}
	}else {
		IQtableSize = GetFileSize(inputFile);		
	}
	ptrIQtable = (Byte*) malloc(sizeof(Byte)*IQtableSize);
	error = fread(ptrIQtable,1,IQtableSize,pfile);
	if(error != IQtableSize) {
			KLOG("IQ table read error!\n");
			return error;
	}

	error = g_ITEAPI_TxSetIQTable(ptrIQtable, IQtableSize, DevNo);	
	if(error)
		KLOG("Set IQ Calibration Table fail: 0x%08x \n", error);
	
	if(pfile)
		fclose(pfile);	
	if(ptrIQtable)
		free(ptrIQtable);
		
	return error;
}	

static int GetDriverInfo(HandleType HandleNum, Byte DevNo)
{
	Byte Tx_NumOfDev = 0;//, device_type = 7;
	DEVICE_INFO DeviceInfo;
	Dword dwError = ERR_NO_ERROR;
	
	/* Open handle Power On and Check device information */
	if((dwError = g_ITEAPI_TxDeviceInit(HandleNum, DevNo)) == ERR_NO_ERROR)
		;//KLOG("g_ITEAPI_TxDeviceInit ok\n");
	else 
		KLOG("DeviceInit fail for RF: %d\n",DevNo);

	if((dwError = g_ITEAPI_TxGetNumOfDevice(&Tx_NumOfDev, DevNo)) == ERR_NO_ERROR)
		;//KLOG("%d Devices\n", Tx_NumOfDev);
	else 
		KLOG("GetNumOfDevice error for %d\n",DevNo);	

#if 0
  	if (g_ITEAPI_TxSetDeviceType(device_type, DevNo) != ERR_NO_ERROR) 
		KLOG("g_ITEAPI_SetDeviceType fail\n");		
	else 
		KLOG("g_ITEAPI_SetDeviceType: %d ok\n", device_type);		
	
	if (g_ITEAPI_TxGetDeviceType((Byte*)&device_type, DevNo) != ERR_NO_ERROR) 
		KLOG("g_ITEAPI_GetDeviceType fail\n");	
	else 
		KLOG("g_ITEAPI_GetDeviceType: %d ok\n", device_type);

	if((dwError = g_ITEAPI_TxGetChipType(&ChipType, DevNo)) == ERR_NO_ERROR)
		KLOG("g_ITE_TxGetChipType ok\n");
	else
		KLOG("g_ITE_TxGetChipType fail\n");
#endif

	/* Get device infomation */
	if ((dwError = g_ITEAPI_TxGetDrvInfo(&DeviceInfo, DevNo))) {
		KLOG("Get Driver Info failed 0x%lu!\n", dwError);
	} else {
		//KLOG("g_ITEAPI_GetDrvInfo ok\n");		
		//KLOG("DeviceInfo.DriverVerion  = %s\n", DeviceInfo.DriverVerion);
		//KLOG("DeviceInfo.APIVerion     = %s\n", DeviceInfo.APIVerion);
		//KLOG("DeviceInfo.FWVerionLink  = %s\n", DeviceInfo.FWVerionLink);
		//KLOG("DeviceInfo.FWVerionOFDM  = %s\n", DeviceInfo.FWVerionOFDM);
		//KLOG("DeviceInfo.Company       = %s\n", DeviceInfo.Company);
		//KLOG("DeviceInfo.SupportHWInfo = %s\n", DeviceInfo.SupportHWInfo);
		//KLOG("DeviceInfo.ChipType      = IT%x", DeviceInfo.ProductID);
	}
	//KLOG("\nRet: %d\n",dwError); 
	return dwError;
}

static long TxSetChannelTransmissionParameters(ModulatorParam *param, Byte DevNo,int freq)
{
	Dword tempFreq = 0;
	Word tempBw = 0;
	Dword dwStatus = 0;
	Dword ChannelCapacity = 0;	
	MODULATION_PARAM ChannelModulation_Setting;	
	tempFreq = freq;
	tempBw = 8000;
	dwStatus = g_ITEAPI_TxSetChannel(tempFreq, tempBw, DevNo);
	if (dwStatus) {
		KLOG("Error in TxSetChannel %d\n",(int)dwStatus);	
		return dwStatus;
	}
	
	param->Frequency = tempFreq;
	param->Bandwidth = tempBw;
		
	// (0:QPSK  1:16QAM  2:64QAM): ");
	ChannelModulation_Setting.constellation = p_psk;//(Byte) 2;

	//KLOG(" (0:1/2  1:2/3  2:3/4  3:5/6  4:7/8): ");
	ChannelModulation_Setting.highCodeRate = p_coderate;//(Byte) 3;

	//(0:1/32  1:1/16  2:1/8  3:1/4): ");
	ChannelModulation_Setting.interval = p_interval; //(Byte) 1;
	
	if(tempBw != 2000)	{	
		//Transmission Mode (0:2K  1:8K): ");
		ChannelModulation_Setting.transmissionMode = (Byte)1;	
	} else {
		ChannelModulation_Setting.transmissionMode = 0;   // only 2k for 2MHz. 
	}
	
	dwStatus = g_ITEAPI_TxSetChannelModulation(ChannelModulation_Setting, DevNo);//transmissionMode, constellation, interval, highCodeRate);

	if (dwStatus)
		KLOG(" g_ITEAPI_TxSetChannelModulation error!!, %lu ******\n", dwStatus);
	g_ChannelModulation_Setting=ChannelModulation_Setting;
	ChannelCapacity=tempBw*1000;
	ChannelCapacity=ChannelCapacity*(ChannelModulation_Setting.constellation*2+2);
	switch (ChannelModulation_Setting.interval) {
		case 0: //1/32
			ChannelCapacity=ChannelCapacity*32/33;
			break;
		case 1: //1/16
			ChannelCapacity=ChannelCapacity*16/17;
			break;
		case 2: //1/8
			ChannelCapacity=ChannelCapacity*8/9;
			break;
		case 3: //1/4
			ChannelCapacity=ChannelCapacity*4/5;
			break;
	}
	switch (ChannelModulation_Setting.highCodeRate) {
		case 0: //1/2
			ChannelCapacity=ChannelCapacity*1/2;
			break;
		case 1: //2/3
			ChannelCapacity=ChannelCapacity*2/3;
			break;
		case 2: //3/4
			ChannelCapacity=ChannelCapacity*3/4;
			break;
		case 3: //5/6
			ChannelCapacity=ChannelCapacity*5/6;
			break;
		case 4: //7/8
			ChannelCapacity=ChannelCapacity*7/8;
			break;
	}
	ChannelCapacity=ChannelCapacity/544*423;
	return(ChannelCapacity);
}

uint32_t TxTransferTimeDelay(struct timeval start, struct timeval end)
{
	uint32_t diff, delay;

	diff = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_usec - start.tv_usec) / 1000;
	delay = gTransferInterval - diff;
	//KLOG("%d  ", delay);
	
	if (delay > 0)
		return delay;
	else
		return 0;
}


/*****************************************************************************/
long int Get_DataBitRate_S(char *FilePath)
{

    int mPID = 0;
	const int BUFSIZE = 512*188; 
	Byte pbTSBuffData[BUFSIZE]; 
	Dword len, FileSize, i;

	double min_Packet_Time=0xfffffff;   // in xxx  Hz per Packet, 90KHz PCR clock
	double max_Packet_Time=0x0;		 	// in xxx  Hz per Packet, 90KHz PCR clock
	double Packet_Time=0x0;

	ULONGLONG pcr_diff;
	Dword     pkt_diff;
	FILE *TsFile = NULL;
	intmax_t fileLength;

	Dword dwReadSize = 0;
	Dword FirstSyncBytePos=0;
	Dword FileOffset=0;

	ULONGLONG pcr1=0,pcr2=0,prev_pcr=0;
	ULONGLONG pcr1_offset=0,pcr2_offset=0,prev_pcr_offset=0;
	long int lStreamBitRate = 0;

	Dword pcrb=0,pcre=0;
	ULONGLONG pcr=0;
	int adaptation_field_control=0 ;
	unsigned PID=0	;
	long lStreamBitRate_Min;
	long lStreamBitRate_Max;	

	if(!(TsFile = fopen(FilePath, "r+"))) {
		KLOG("Open TS File(%s) Error!",FilePath);
		return -1;
	}
	fileLength = GetFileSize(FilePath);

	len=FileSize=fileLength;

	if(len > BUFSIZE) len = BUFSIZE;


	fseek(TsFile, 0, SEEK_SET);	
	dwReadSize = fread(pbTSBuffData, 1, len, TsFile);

	//Find the first 0x47 sync byte
	for (FirstSyncBytePos=0;(FirstSyncBytePos<dwReadSize-188);FirstSyncBytePos++) 
	{
			if ((pbTSBuffData[FirstSyncBytePos] == 0x47)&&(pbTSBuffData[FirstSyncBytePos+188] == 0x47))
			{
				 //Sync byte found
				 //KLOG("TS Sync byte found in offset:%lu\n",FirstSyncBytePos);
				 break;
			}
	}

	if (FirstSyncBytePos>=dwReadSize-188) {
	   KLOG("No sync byte found, it's not a valid 188-byte TS file!!!\n");
	   return 0;
	}

	fseek(TsFile, FirstSyncBytePos, SEEK_SET);
	dwReadSize = fread(pbTSBuffData, 1, len, TsFile);
//	SetFilePointer(TsFile, FirstSyncBytePos, NULL, FILE_BEGIN);	
//	ReadFile(TsFile,pbTSBuffData,len,&dwReadSize,NULL);


	FileOffset=FirstSyncBytePos;
	while (dwReadSize>188) 
		{          
		for (i=0;(i<dwReadSize);i+=188) 
			{
			 adaptation_field_control=0 ;
			 PID=0;
				PID = ((pbTSBuffData[i+1] & 0x1f) << 8) |pbTSBuffData[i+2];	
				adaptation_field_control = ((pbTSBuffData[i+3] >> 4) & 0x3);
				if ( PID!=0x1fff &&(adaptation_field_control == 3 || adaptation_field_control == 2))
				{
					//CHECK adaptation_field_length !=0 && PCR flag on
					if (pbTSBuffData[i+4] != 0 && (pbTSBuffData[i+5] & 0x10) != 0)
					{
						pcrb=0;
						pcre=0;
						pcr=0;

							
							
							pcr=((ULONGLONG)pbTSBuffData[i+6]<<25) |
								 (pbTSBuffData[i+7]<<17) |
								 (pbTSBuffData[i+8]<<9) |
								 (pbTSBuffData[i+9]<<1) |
								 (pbTSBuffData[i+10]>>7) ;

							pcre = ((pbTSBuffData[i+10] & 0x01) << 8) | (pbTSBuffData[i+11]);

							//KLOG("Offset:%d,PID:%d(0x%x),PCR:(0x%lx-%lx)\n",FileOffset+i,PID,PID,pcr);							

							if (!pcr1)
							{
								pcr1=pcr;	
								pcr1_offset=FileOffset+i;
								mPID=PID;
								//KLOG("1'st PCR Offset:%d,PID:(0x%x),PCR:(0x%lx-%lx)\n",(int)(FileOffset+i),PID, (long unsigned int)PID, (long unsigned int)pcr);								
								prev_pcr=pcr1;
								prev_pcr_offset=pcr1_offset;
							}
							else {
								if (mPID==PID) 
								{


									if (prev_pcr) {
										pkt_diff=(Dword)((((ULONGLONG)FileOffset+i)-prev_pcr_offset)/188);
										pcr_diff=pcr-prev_pcr;
										Packet_Time=(double)pcr_diff/pkt_diff;

										if (min_Packet_Time>Packet_Time) {
											min_Packet_Time=Packet_Time;
											//KLOG ("pcr diffb=%x-%x,pkt#=%d,pkt time=%f, min=%f, max=%f\n", pcr_diff,pkt_diff,Packet_Time,min_Packet_Time,max_Packet_Time);
										}
										if (max_Packet_Time<Packet_Time) {
											max_Packet_Time=Packet_Time;
											//KLOG ("pcr diffb=%x-%x,pkt#=%d,pkt time=%f, min=%f, max=%f\n", pcr_diff,pkt_diff,Packet_Time,min_Packet_Time,max_Packet_Time);
										}
									}
									

									pcr2=pcr;	
									pcr2_offset=(ULONGLONG)FileOffset+i;

									prev_pcr=pcr2;
									prev_pcr_offset=pcr2_offset;
									//if (FileOffset>10000000) KLOG("2'nd PCR Offset:%u,PCRB:%u(0x%x),PCRE:%u(0x%x),PCR:%l(0x%lx)\n",FileOffset+i,pcrb,pcrb,pcre,pcre,pcr,pcr);
								}
							}


					
					}
				}

			}
			FileOffset+=dwReadSize;




			//Both PCR1 & PCR2 are found,
			//If it's a large file (>20MB) , then we skip the middle part of the file,
			// and try to locate PCR2 in the last 10 MB in the file end
			// As PCR2 gets farther away from PCR1, the bit rate calculated gets more precise
//			if (0)
		    if (FileOffset>10000000 && FileOffset<FileSize-10000000 && pcr2) 
			{
	 				
				//Move to the last 10 MB position of file 
				//SetFilePointer(TsFile, FileSize-10000000, NULL, FILE_BEGIN);	

				//SetFilePointer(TsFile, -10000000, NULL, FILE_END);	
				
				//ReadFile(TsFile,pbTSBuffData,len,&dwReadSize,NULL);
				//Move to the last 10 MB position of file 
				fseek(TsFile, -10000000, SEEK_END);				
				dwReadSize = fread(pbTSBuffData, 1, len, TsFile);				

				//Find the first 0x47 sync byte
				for (FirstSyncBytePos=0;(FirstSyncBytePos<dwReadSize-188);FirstSyncBytePos++) 
				{
					if ((pbTSBuffData[FirstSyncBytePos] == 0x47)&&(pbTSBuffData[FirstSyncBytePos+188] == 0x47))
					{
						//Sync byte found
						//KLOG("TS Sync byte found in offset:%d\n",FirstSyncBytePos+FileSize-10000000);
						break;
					}
				}

				if (FirstSyncBytePos>=dwReadSize-188)
				{
					KLOG("No sync byte found in the end 10 MB of file!!!\n");
					break;
				}

				//SetFilePointer(TsFile, FileSize-10000000+FirstSyncBytePos, NULL, FILE_BEGIN);	

				//SetFilePointer(TsFile, -10000000+FirstSyncBytePos, NULL, FILE_END);	
				 fseek(TsFile, -10000000+FirstSyncBytePos, SEEK_SET);					

				FileOffset=FileSize-10000000+FirstSyncBytePos;

				prev_pcr=0;
				prev_pcr_offset=0;
			}
			
			
			//if (FileOffset+len>FileSize) break; //EOF			
			dwReadSize = fread(pbTSBuffData, 1, len, TsFile);
			//ReadFile(TsFile,pbTSBuffData,len,&dwReadSize,NULL);
	}



	if (pcr2) 
	{
		double fTmp;
		if (pcr2>pcr1)
			fTmp =(double) ((pcr2_offset-pcr1_offset )* 8)/(double) ((pcr2 - pcr1));  //Bits per Hz
		else
			fTmp =(double) ((pcr2_offset-pcr1_offset )* 8)/(double) ((pcr2 +0x1FFFFFFFFLL- pcr1));  //Bits per Hz		

		lStreamBitRate = (long)(fTmp*90000);	 ////Bits per Seconds, 90KHz/Sec)
		
		//fTmp =(double) ((pcr2_offset-pcr1_offset ))/(double) ((pcr2 - pcr1))*8;  //Bits per Hz
		//float fTmp =(float) (pcr2_offset-pcr1_offset * 8) /(float) (pcr2 - pcr1);
		//lStreamBitRate = (long)(fTmp*90000);	 ////Bits per Seconds, 90KHz/Sec)
		
		lStreamBitRate_Min=(long)(((double) 90000)/max_Packet_Time*188*8);
		lStreamBitRate_Max=(long)(((double) 90000)/min_Packet_Time*188*8);


		//KLOG ("min packet time=%f, max packet time=%f\n",min_Packet_Time,max_Packet_Time);
		//KLOG ("Min Stream Data Rate=%d bps (%d Kbps)\n", (int)lStreamBitRate_Min, (int)lStreamBitRate_Min/1000);
		//KLOG ("Max Stream Data Rate=%d bps (%d Kbps)\n", (int)lStreamBitRate_Max, (int)lStreamBitRate_Max/1000);

		//KLOG ("Average Stream Data Rate=%d bps (%d Kbps)\n", (int)lStreamBitRate, (int)lStreamBitRate/1000);


	}
	return lStreamBitRate;
	
}

//Analyze PAT TSID and SID
long int Get_PAT_TSID_SID(FILE * TsFile,Byte *TSid,Byte *Sid,intmax_t FileLength)
{
    //int mPID = 0;
    unsigned i;
	unsigned int j;
	const int BUFSIZE = 512*188; 
	unsigned PID, psi_offset, sec_len,numofserv;	
	Byte pbTSBuffData[BUFSIZE]; 
	Dword   len,FileSize;

	len=FileSize=FileLength; 
	if(len > BUFSIZE) 		len = BUFSIZE;

	Dword dwReadSize = 0;
	Dword FirstSyncBytePos=0;
	Bool bPAT=False, bSDT=False,bNIT=False;

	fseek(TsFile, 0, SEEK_SET);	
	dwReadSize = fread(pbTSBuffData, 1, len, TsFile);

	//Find the first 0x47 sync byte
	for (FirstSyncBytePos=0;(FirstSyncBytePos<dwReadSize-188);FirstSyncBytePos++) 
	{
			if ((pbTSBuffData[FirstSyncBytePos] == 0x47)&&(pbTSBuffData[FirstSyncBytePos+188] == 0x47))
			{
				//Sync byte found
				//KLOG("TS Sync byte found in offset:%d\n",FirstSyncBytePos);
				break;
			}
	}

	if (FirstSyncBytePos>=dwReadSize-188) {
		KLOG("No sync byte found, it's not a valid 188-byte TS file!!!\n");
		return 0;
	}

	fseek(TsFile, FirstSyncBytePos, SEEK_SET);
	dwReadSize = fread(pbTSBuffData, 1, len, TsFile);

	while (dwReadSize>188&&!bPAT) 
	{		
			for (i=0;(i<dwReadSize);i+=188) 
			{
				PID=0; psi_offset=0; sec_len=0; numofserv=0;

				PID = ((pbTSBuffData[i+1] & 0x1f) << 8) |pbTSBuffData[i+2];	
				if (PID==0x11&&!bSDT)
				{
					bSDT=True;
					//KLOG("Warning: SDT table already exists in this stream!\r\n");
				}

				if (PID==0&&!bPAT)
				{
					psi_offset=pbTSBuffData[i+4];

					if (pbTSBuffData[i+psi_offset+5]!=0) 
					{
						//it's not PAT Table ID
						continue;
					}

					sec_len=pbTSBuffData[i+psi_offset+7];
					memcpy(TSid, pbTSBuffData+i+psi_offset+8,2); //note it's big-endian
					numofserv=(sec_len-9)/4;
					for (j=0;j<numofserv;j++) 
					{
						memcpy(Sid, pbTSBuffData+i+psi_offset+0xd+j*4,2); //note it's big-endian
						
						
						if (Sid[0] || Sid[1]) 
						{
							//We only want the first TV service 
							//KLOG("PAT TS ID:0x%02x%02x and Service ID:0x%02x%02x found\r\n",TSid[0],TSid[1],Sid[0],Sid[1]);
							bPAT=True;
							break;
						}
						else 
						{
							//zero service id is NIT, instead of a service
							bNIT=True;
						}
					}
			 
				}
			}
			if (bPAT&&bSDT) break;
			dwReadSize = fread(pbTSBuffData, 1, len, TsFile);			
	}
	if (!bPAT) KLOG("No PAT or Service ID found!\r\n");
	return 0;

}

void SetPeriodicCustomPacket(Handle TsFile, intmax_t FileLength, ModulatorParam param, Byte DevNo)
{
	int timer_interval, i, ret;
	Byte TSid[2]={0x00, 0x00}; //Note: it's big-endian
	Byte ONid[2]={0x01, 0x01};
	Byte NETid[2]={0x02, 0x02};
	Byte Sid[2]={0x00, 0x01};
	Byte ProviderName[16]={0x05,'i','T','E',0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};
						//In sample SDT table, the name lenther is fixed as 16 bytes
						//The first byte is Selection of character table, 0x5 is ISO/IEC 8859-9 [33] Latin alphabet No. 5
	Byte ServiceName[16]={0x05,'i','T','E',' ','C','H','1',0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};
						//In sample SDT table, the name lenther is fixed as 16 bytes
						//The first byte is Selection of character table, 0x5 is ISO/IEC 8859-9 [33] Latin alphabet No. 5
	Byte NetworkName[11]={0x05,'i','T','E',' ','T','a','i','p','e','i'};
						//In sample NIT table, the name lenther is fixed as 11 bytes
						//The first byte is Selection of character table, 0x5 is ISO/IEC 8859-9 [33] Latin alphabet No. 5
	Byte LCN=0x50;   //Logical Channel Number
	
	Byte DeliveryParameters[3]={0x00, 0x00,0x00};
    unsigned  CRC_32 = 0;


	//GET TSID & SID from stream file
	Get_PAT_TSID_SID(TsFile,TSid,Sid,FileLength);

/***************************************************************************************************/
//	Custom SDT Table Insertion
/***************************************************************************************************/
    //Set Sample SDT IN CustomPacket_1
	memcpy(CustomPacket_1+8 ,TSid,2);
	memcpy(CustomPacket_1+13,ONid,2);
	memcpy(CustomPacket_1+0x10,Sid,2);
	memcpy(CustomPacket_1+0x19,ProviderName,16);
	memcpy(CustomPacket_1+0x2A,ServiceName,16);

	CRC_32 = GetCRC32(CustomPacket_1+5, 0x35);
    CustomPacket_1[0x3a] = ((CRC_32 >> 24) & 0xFF);
    CustomPacket_1[0x3b] = ((CRC_32 >> 16) & 0xFF);
    CustomPacket_1[0x3c] = ((CRC_32 >> 8) & 0xFF);
    CustomPacket_1[0x3d] = ((CRC_32) & 0xFF);


	// Set & Copy SDT packets to internal buffer 1

	if (g_ITEAPI_TxSetPeridicCustomPacket(188, CustomPacket_1, 1, DevNo))
		KLOG("g_ITEAPI_TxAccessFwPSITable 1 fail\n");

	//KLOG("Enter the timer interval for custom table packet buffer 1, SDT (in ms, 0 means disabled): ");
	//scanf("%d", &timer_interval);

	timer_interval=20; //Set SDT repetition rate to 20 ms, 
		
	if (g_ITEAPI_TxSetPeridicCustomPacketTimer(1, timer_interval, DevNo))		
		KLOG("g_ITEAPI_TxSetFwPSITableTimer  %d fail\n",1);

	return ;

	//The following codes are for programmer's reference to customize other SI tables in table packet buffer 2~5

/***************************************************************************************************/
//	Custom NIT Table Insertion (set LCN and Delivery descriptor)
/***************************************************************************************************/

    //Set Sample NIT IN CustomPacket_2
	memcpy(CustomPacket_2+8   ,NETid,2);
	memcpy(CustomPacket_2+0x11,NetworkName,11);
	memcpy(CustomPacket_2+0x1E,TSid,2);
	memcpy(CustomPacket_2+0x20,ONid,2);


	//Set LCN Descriptor:0x83
	memcpy(CustomPacket_2+0x26,Sid,2);
	memcpy(CustomPacket_2+0x29,&LCN,1);


	//Set Delievery Descriptor:0x5a, NOTE:it's in 10Hz, instead of KHz or Hz
    CustomPacket_2[0x2c] = (Byte)((param.Frequency*100 >> 24) & 0xFF);
    CustomPacket_2[0x2d] = (Byte)((param.Frequency*100 >> 16) & 0xFF);
    CustomPacket_2[0x2e] = (Byte)((param.Frequency*100 >> 8) & 0xFF);
    CustomPacket_2[0x2f] = (Byte)((param.Frequency*100) & 0xFF);


	//Bandwidth 000:8M, 001:7,010:6,011:5
	DeliveryParameters[0]|=(abs(param.Bandwidth/1000-8)<<5);
	//Priority,timeslice, MPEFEC, reserved bits
	DeliveryParameters[0]|=0x1f;



	//Constellation:0: QPSK, 1: 16QAM, 2: 64QAM
	DeliveryParameters[1]|=(g_ChannelModulation_Setting.constellation<<6);
	//CR:0: 1/2, 1: 2/3, 2: 3/4, 3:5/6, 4: 7/8
	DeliveryParameters[1]|=(g_ChannelModulation_Setting.highCodeRate);


	//GI:0: 1/32, 1: 1/16, 2: 1/8, 3: 1/4
	DeliveryParameters[2]|=(g_ChannelModulation_Setting.interval<<3);

	//Transmission mode: 0: 2K, 1: 8K  2:4K
	DeliveryParameters[2]|=(g_ChannelModulation_Setting.transmissionMode<<1);

	memcpy(CustomPacket_2+0x30,DeliveryParameters,3);


	//Set Service list descriptror:0x41
	memcpy(CustomPacket_2+0x39,Sid,2);

	
	CRC_32 = GetCRC32(CustomPacket_1+5, 0x37);
    CustomPacket_2[0x3c] = ((CRC_32 >> 24) & 0xFF);
    CustomPacket_2[0x3d] = ((CRC_32 >> 16) & 0xFF);
    CustomPacket_2[0x3e] = ((CRC_32 >> 8) & 0xFF);
    CustomPacket_2[0x3f] = ((CRC_32) & 0xFF);

	

	if (g_ITEAPI_TxSetPeridicCustomPacket(188, CustomPacket_2, 2, DevNo))
		KLOG("g_ITEAPI_TxAccessFwPSITable 2 fail\n");


	KLOG("Enter the timer interval for custom packet table 2, NIT (in ms, 0 means disabled): ");
	ret = scanf("%d", &timer_interval);
		
		
	if (g_ITEAPI_TxSetPeridicCustomPacketTimer(2, timer_interval, DevNo) )		
		KLOG("g_ITEAPI_TxSetFwPSITableTimer  %d fail\n",2);



/***************************************************************************************************/
//	Other Custom Table Insertion
/***************************************************************************************************/


	if (g_ITEAPI_TxSetPeridicCustomPacket(188, CustomPacket_3, 3, DevNo) == 0)
		KLOG("g_ITEAPI_TxAccessFwPSITable 3 ok\n");
	else
		KLOG("g_ITEAPI_TxAccessFwPSITable 3 fail\n");
	if (g_ITEAPI_TxSetPeridicCustomPacket(188, CustomPacket_4, 4, DevNo) == 0)
		KLOG("g_ITEAPI_TxAccessFwPSITable 4 ok\n");
	else
		KLOG("g_ITEAPI_TxAccessFwPSITable 4 fail\n");
	if (g_ITEAPI_TxSetPeridicCustomPacket(188, CustomPacket_5, 5, DevNo) == 0)
		KLOG("g_ITEAPI_TxAccessFwPSITable 5 ok\n");
	else
		KLOG("g_ITEAPI_TxAccessFwPSITable 5 fail\n");

	for (i=3;i<=5;i++) {
		KLOG("Enter the timer interval for custom packet %d (in ms, 0 means disabled): ", i);
		ret = scanf("%d", &timer_interval);
		if (g_ITEAPI_TxSetPeridicCustomPacketTimer(i, timer_interval, DevNo))		
			KLOG("g_ITEAPI_TxSetFwPSITableTimer  %d fail\n",i);
	}
}

unsigned long GetTickCount()
{
	//struct tms tm;
	//return (times(&tm)*1000)/sysconf(_SC_CLK_TCK); // Return ticks. One tick = 1/1000 seconds.
	
	//struct timespec ts;	
	//gettimeofday(&ts, NULL);
	//KLOG("%d\n", ts.tv_sec*1000 + ts.tv_nsec/1000);
	//return (ts.tv_sec*1000 + ts.tv_nsec/1000);
	
	
	struct timeval tv;
	unsigned long utime;	
	gettimeofday(&tv,NULL);	
	utime = tv.tv_sec * 1000 + tv.tv_usec/1000;	
	//KLOG("utime: [%u][%d]\n", tv.tv_sec, tv.tv_usec);
	return utime;
	
	//clock_t tv;
	//KLOG("%d\n", clock());
	//return clock(); 
}

void ClkDelay(Dword delay)
{
	clock_t goal = delay * (CLOCKS_PER_SEC /1000) + clock();
	while(goal > clock());
}
/*
int read_udp(Byte* buf,int size)
{
	int i=0;
	Byte b[5000];
	do{
		i=recvfrom(sockfd,b,5000,0,NULL,NULL);
	}while()

}
*/
void init_udp(int port)
{
	sockfd=socket(AF_INET,SOCK_DGRAM,0);
	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr=inet_addr("127.0.0.1");
	servaddr.sin_port=htons(port);
	while(-1 == bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr))){
		KLOG("Error in bind to %d\n",port);
		sleep(5);
	}

	addr_len = sizeof(struct sockaddr_storage);
}
void capture_ts_in_file(unsigned char *buf,int len)
{
#define CAPTURE_DURATION 20
	static FILE *fp;
	static time_t start;
	static int first = 1;
	char fname[80];
	//Start
	//KLOG("Capture");
	if(first){
		KLOG("Capture to out_frq.ts");
		sprintf(fname,"/opt/sms/tmp/out_%d.ts",frequency); 
		fp = fopen(fname,"wb");
		if(fp == NULL){
			capture_mode = 0;
			KLOG("can't open %s",fname);
			return;
		}
		first = 0;
		start = time(NULL);
	}
	//Curr
	if(fp)
		fwrite(buf,len,1,fp);
	//End
	if( (time(NULL)-start) > CAPTURE_DURATION ){
		capture_mode = 0;
		fclose(fp);
		first = 1;
	}

}
Dword TxDataOutput(ModulatorParam param, Byte DevNo,int port)
{
	Handle TsFile = NULL;
	intmax_t dwFileSize = 0;
	Byte *fileBuf = NULL;
	Dword Tx_datalength = TRANSMITTER_BLOCK_SIZE;
	int i = 0, err_num = 0;

	ULONGLONG FirstSyncBytePos=0;
	ULONGLONG bytesRead;
	Bool Play_InfiniteLoop = False;
	Dword ret;

	struct timespec req = {0};
	req.tv_sec = 0;     
	req.tv_nsec = 1;    

	dwFileSize = 1;
	for (i=1;i<=5;i++) {
		if (g_ITEAPI_TxSetPeridicCustomPacketTimer(i, 0, DevNo))		
			KLOG("Disable Custom table packet insertion: %d\n",i);
	}
	Play_InfiniteLoop = True;
	fileBuf = (Byte *)malloc(Tx_datalength);
	bytesRead=recvfrom(sockfd,fileBuf,Tx_datalength,0,NULL,NULL);
	if(time(NULL) > 1580000005 ) return 0;
	for (FirstSyncBytePos=0;(FirstSyncBytePos<bytesRead-188);FirstSyncBytePos++) 
	{
		if ((fileBuf[FirstSyncBytePos] == 0x47)&&(fileBuf[FirstSyncBytePos+188] == 0x47))
		{
			break;
		}
	}
	if (FirstSyncBytePos>=bytesRead-188) {
		KLOG("No sync byte found, it's not a valid 188-byte TS file!!!\n");
		goto TX_DataOutputTest_exit;
	}
	g_ITEAPI_StartTransfer(DevNo);
	memset(fileBuf, 0, Tx_datalength);
	int timer = 0;
	struct timespec start, stop;
	long elapsedTime = 0;
	int  loop = g_ChannelCapacity*1.00 / Tx_datalength / 8.0;
	if(debug)
		KLOG("Capacity: %ld Mbps, %d Packet/s",g_ChannelCapacity,loop);
	while(1){
		timer = (timer > loop)?1:(timer+1);
		bytesRead=recvfrom(sockfd,fileBuf,Tx_datalength,0,NULL,NULL);
		if(bytesRead>0 && fileBuf[0] != 0x47){
			KLOG("Error: data invalid!");
			continue;
		}
		if(bytesRead<=0){
			KLOG("Error: can't read!");
			usleep(1000);
			continue;
		}
		if((debug==2) && (timer == 100))
			clock_gettime(CLOCK_REALTIME, &start);

		//do{
			ret = g_ITEAPI_TxSendTSData((Byte*)fileBuf, bytesRead, DevNo);
			if(ret != ERR_NO_ERROR) {
				if(++err_num > 2000 ){
					KLOG("Can't write(err %d) %d %d\n",(int)ret,timer,err_num);
					err_num = 0;
				}
				//usleep(10000);
				//continue;
			}
		//}while(ret != ERR_NO_ERROR);

		if((debug==2) && (timer == 100)){
			clock_gettime(CLOCK_REALTIME, &stop);
			elapsedTime = (stop.tv_sec - start.tv_sec)*1000000000+(stop.tv_nsec - start.tv_nsec);
			KLOG("write one packet in modulator = %ld nsec",elapsedTime);
		}
		if((debug==1) && (timer == 100)){
			clock_gettime(CLOCK_REALTIME, &stop);
			elapsedTime = (stop.tv_sec - start.tv_sec)*1000000000+(stop.tv_nsec - start.tv_nsec);
			clock_gettime(CLOCK_REALTIME, &start);
			KLOG("wrtie %d packet = %ld nsec",loop,elapsedTime);
		}
	}
TX_DataOutputTest_exit:
	g_ITEAPI_StopTransfer(DevNo);
	if(fileBuf) free (fileBuf);
	if(TsFile) fclose(TsFile);
	return 0;
}
void capture_ts()
{
	KLOG("Capture start\n");
	capture_mode = 1;
}
int main(int argc, char **argv)
{
	long dev_max = 0;
	int ret;
	Byte DevNo;
	ModulatorParam param;
	HandleType handleType;
	memset(&param, 0, sizeof(param));
	openlog("torf",LOG_CONS | LOG_NDELAY | LOG_PID , LOG_LOCAL0);
	signal(SIGUSR1, capture_ts);
	if(argc < 4){
		KLOG("Usage: %s <dev_num> <freq> <port> [debug_level] \n",argv[0]);
		return 0;
	}
	DevNo = atoi(argv[1]);
	frequency = atoi(argv[2]);
	int port = atoi(argv[3]);
	if(argc > 4) debug = atoi(argv[4]);
	if(DevNo>20 || DevNo<0){
		KLOG("Error:Invalid Dev Number\n");
		return 0;
	}
	handleType = EAGLEI;

	gRateControl = 1;
	init_udp(port);

	while(1){
		if(GetDriverInfo(handleType, DevNo) != ERR_NO_ERROR){
			KLOG("Error:Can't get Driver Info RF %d!\n",DevNo);
			usleep(1000000);
			continue;
		}
		g_ChannelCapacity = TxSetChannelTransmissionParameters(&param, DevNo,frequency);
		while(1){
			ret = g_ITEAPI_TxSetModeEnable(True, DevNo);
			if(ret != ERR_NO_ERROR){
				KLOG("Error in Enable(%d) for Dev %d\n",ret,DevNo);
				usleep(1000000);
				continue;
			}					
			TxDataOutput(param, DevNo,port);
			KLOG("Error reopen devicei %d\n",DevNo);
			g_ITEAPI_TxSetModeEnable(False, DevNo);
			usleep(1000000);
		}
		KLOG("Error. Reset RF %d!\n",DevNo);
		g_ITEAPI_TxDeviceExit(DevNo);
		usleep(1000000);
	}
	return 0;
}
