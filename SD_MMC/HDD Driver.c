//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//	Name:			   HDD Driver.c                       					  //
//	Date:			   02/01/2004		                   					  //
//	Version:		   1.1											              //
//	Type:			   PIC C Driver for MMC										  //
//	Author:        Mike Luck & Douglas Kennedy				           //
//	Company:		   MPIC3.COM										           //
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//	Note:
// 		This code is a work in progress & not a finished fully working
//			project. Please check www.mpic3.com reguarly for updates.

/// PIN Assignments //////////////////////////////
#DEFINE _CS PIN_C2 // chip select for MMC
//#DEFINE SDO PIN_C5
//#DEFINE CLK PIN_C3
//#DEFINE SDI PIN_C4
// SPI hardware pins are
// SDO C5
// SDI C4
// SCK C3
///////////////////////////////////////////////////////////////

///// Note old values needed before all SPI modes could be set up using SPI_setup
/// for 16 parts ///////////
//#DEFINE SSPCON 0x14
//#DEFINE SSPSTAT 0x94
//#BIT SMP=SSPSTAT.7
//#BIT CKE=SSPSTAT.6
//#BIT CKP=SSPCON.4
//#BIT SSPEN=SSPCON.5
//////////////////////////////

// For 18F452
#DEFINE SSPSTAT 0x0FC7
#DEFINE SSPCON1 0x0FC6
#BIT SMP=SSPSTAT.7
#BIT CKE=SSPSTAT.6
#BIT CKP=SSPCON1.4

#DEFINE MAX_FILES 2 /// max number of open files
#DEFINE MMC_BUFF_SIZE 32 /// 32 for PCM
#DEFINE MMC_FILE_NAME_SIZE 32

#DEFINE ROOT_CLUSTER 0
#DEFINE NEXT_CLUSTER 1

#define MMC_INIT_TRACE FALSE
#define MMC_CMD_TRACE FALSE
#define MMC_CLUSTER_TRACE FALSE // if true prints to serial port
#define MMC_OPEN_TRACE FALSE // if true prints to serial port
#define MMC_READ_TRACE FALSE // if true prints file_addr,cluster index etc
#define MMC_WRITE_TRACE FALSE
#define MMC_READ_BLOCK_TRACE FALSE
#define MMC_SET_BLOCK_LEN_TRACE FALSE
#define MMC_WRITE_BLOCK_TRACE FALSE
#define MMC_NEW_CLUSTER FALSE
////// MMC prototypes
#separate
int init_MMC(int max_tries);
#separate
int open_file(int fnbr,char *fname,int16 rec_length);
#separate
int file_read(int8 fnbr,char *buff);
#separate
int file_write(int8 fnbr,int *buff);
#separate
int file_set(int fnbr,int32 offset);
#separate
int file_new_cluster(int8 fnbr,int8 mode); /// mode 1=fat1 2=fat2

int32 atoint32 (char *s );
signed int strncmp(char *s1, char *s2, int n);
///////////////////// MMC GLOBALS /////////////////////////////
int16 cluster_size_bytes; // bytes in a cluster
//int16 dir_cluster_chain_ptr; // link to the first cluster in the dir

int32 fat1_address; // physical address of fat1 cluster table assigned by INIT_MMC
int32 fat2_address; // physical address of fat1 cluster table assigned by INIT_MMC
int32 root_dir_address; // physical address of volume,file,folder tiles assigned by INIT_MMC
int32 data_area_address; // physical address of data area assigned by INIT_MMC
int32 winhex_adj; // Win hex hides the bytes in the reserved sectors
// this means Fat1 is address 512
// so adj is fat1-512

int32 block_size; // current MMC block size

int MMC_init=FALSE;

int MMC_dir_protected=TRUE;

////////// open file specific globals ///////////////////////
struct{
	char name[MMC_FILE_NAME_SIZE+1]; // fopen file name
	int32 dir_addr_ptr; // physical address of this files tile info
	int16 root_cluster_ptr; // location of first cluster in FAT
	int16 this_cluster_ptr; // location of current cluster in FAT
	int16 next_cluster_ptr; // location of the next cluster for a file or sub dir in FAT
	int32 addr_ptr; // physical address in the file the current
	// cluster points to
	// address=(this_chain_ptr-2)*cluster_size_bytes+data_area_address
	//
	// cluster_addr(THIS_CLUSTER) assigns it
	// cluster_addr(NEXT_CLUSTER) moves to the data the next
	// cluster points to
	int32 size; // size of open file in bytes
	int32 cluster_offset; // offset within the file representing the start of the current cluster
	// (0 is start and ends with the cluster contianing eof )
	// auto increased by cluster_size_bytes each time a new cluster is entered

	int32 offset; // current offset into the open file ( 0 is start size(file size) is end)
	// auto increased by rec size each time a rec is read
	// addr_prt+offset-cluster_offset is physical address of
	// the current position within the file
	// the physical positions are not always contiguous since the
	// clusters of the file are not always adjacent to each other
	int16 rec_size; // fopen record_size
	// char buff[MMC_BUFF_SIZE+1]; // used for open and for read write
	// init MMC uses file 0 buff to fetch the globals

} file[MAX_FILES];


#separate
int mmc_cmd(int8 cmd,int32 address,int8 tries,int8 valid,int8 invalid){
	int i,r1;
	for( i=0;i<16;i++) SPI_READ(0xFF);// digest prior operation
	// commands
	// 7 6 5 4 3 2 1 0
	// 0 1 b b b b b b bbbbbb=cmd
	// 16=0x50 set blocklength
	// 17=0x51 read block
	// 24=0x58 write block
	#if MMC_CMD_TRACE
	printf("\n\r cmd=%2X \n\r",cmd);
	#endif
	SPI_READ(cmd);
	SPI_READ(MAKE8(address,3));
	SPI_READ(MAKE8(address,2));
	SPI_READ(MAKE8(address,1));
	SPI_READ(MAKE8(address,0));
	SPI_READ(0x95); // valid crc for 0x40 only invalid for others but spi mode doesn't care
	for(i=0;i< tries;i++) {
		r1=SPI_READ(0xFF);
		#if MMC_CMD_TRACE
			printf(" %2X",r1);
		#endif
		if (r1==valid) break;
		if (r1==invalid) break;
	}
	return(r1);
}


#separate
int set_BLOCKLEN( int32 size){
	int r1;

	r1=mmc_cmd(0x50,size,16,0x00,0x40); /// cmd.data,tries,valid code,invlaid code
	if (r1==0x00) goto done ;
	if (r1==0x40) goto invalid;


	return(false);
	invalid:
	#IF MMC_SET_BLOCK_LEN_TRACE
		printf("\n\r para err\n\r");
	#ENDIF
done:
	block_size=size; //// assign global block size
	//printf("\n\r blk size=%lu",block_size);
	return(true);
}


#separate
int read_BLOCK( int32 address, char *buff){
	//// low level read ..requires block len to be called first to set global blocksize
	int r1;
	long i,iw; /// allows large gt 255 buff size addressing
	//int data[128];
	r1=mmc_cmd(0x51,address,16,0x00,0x40);

	if (r1==0x00) goto get_token ; // we can read data payload
	if (r1==0x40) goto invalid;

	#IF MMC_READ_BLOCK_TRACE
		printf("\n\r read block err 1 address=%lu \n\r",address);
	#ENDIF
	return(false);
	invalid:
	#IF MMC_READ_BLOCK_TRACE
		printf("\n\r read block err 2 adress=%lu \n\r",address);
	#ENDIF
	return(false);
	get_token:
	for(iw=0;iw<1024;iw++){
	r1=SPI_READ(0xFF);
	//data[iw]=r1;
	if (r1==0xFE) goto read_data; // read token $FE
	}
	#IF MMC_READ_BLOCK_TRACE
		printf("\n\r read block err 3 address=%lu \n\r",address);
	#ENDIF
	return(false);
	read_data:
	#IF MMC_READ_BLOCK_TRACE
		printf("\n\r read block tries for FE =%lu \n\r",iw);
	#ENDIF

	for (i=0;i<block_size;i++) buff[i]=SPI_READ(0xFF);
	SPI_READ(0xFF); // read crc
	SPI_READ(0xFF);

	return(true);
}

//////////////////////////////////////////////////////////////////
///////////////////////////////// INIT MMC ///////////////////////
//////////////////////////////////////////////////////////////////
#separate

int init_MMC(int max_tries){
	int32 start_lsec;
	int16 sec_resv,sec_for_FAT,bytes_per_sector,root_dir_entries,
	sec_for_data,count_of_clusters,root_dir_sectors,total_sectors;
	int i,tries,sec_per_cluster,c;
	char buff[32];
	tries=0;
	cmd0:
	///////////////////// place null treminators in globals fname and buff
	for(i=0;i<MAX_FILES;i++){
		file[i].name[0]=0;
		file[i].rec_size=32; //// default rec_size = 32 byte tile size of FAT16
	}
	//buff[MMC_BUFF_SIZE]=0;
	//frec_size=32; //// default rec_size = 32 byte tile size of FAT16
	output_high(_CS); /// reset chip hardware !!! required
	delay_ms(20);
	for(i=0;i<20;i++) SPI_READ(0xFF); // min 80 clocks to get MMC ready
	output_low(_CS); /// !!! required
	delay_ms(20);
	#if MMC_INIT_TRACE
		printf("cmd0");
	#ENDIF
	c=mmc_cmd(0x40,0x00000000,128,0x01,0x99);

	if (c==0x01) goto exit_cmd1;

	// note: i must cycle at least 8 times (16 is safe )

	if (tries++<max_tries) goto cmd0; /// restart
	else return (10);
	exit_cmd1:
	// CPDMOD - This SOMETIMES seems to be necessary
	// output_high(_CS);
	// SPI_READ(0xFF); // min 8 clocks to get MMC ready
	// output_low(_CS);
	//CPDMOD End


	tries=0;
	cmd1:

	/// now try to switch to idle mode
	/// Note: cmd1(idle) is the only command allowed after a cmd0(reset)
	//

	c=mmc_cmd(0x41,0x00000000,128,0x00,0x99);
	if (c==0x00) { goto ready;}


	if( tries++<max_tries) { printf("cmd1"); goto cmd1;}
	else return(11);
	ready:
	//for( i=0;i<32;i++) SPI_READ(0xFF);// digest operation
	/// MMC is inialized and in idle state ready for commands
	////
	//// we need to first access the master boot sector physical address=0
	///
	if(set_BLOCKLEN((int32)32)==false) return(12); /// sets global block_size to 32

	if (read_block(0x00000000,buff)==false) return (99); /// read the first few bytes
	#if MMC_INIT_TRACE
		printf("\n\r sector0=");
		for(i=0;i<32;i++)printf("%2X ",buff[i]);
	#ENDIF
	if (buff[0]==0xEB || buff[0]==0xE9){
		/// sector 0 is the boot sector
		#if MMC_INIT_TRACE
			printf("\n\r boot sector= 0");
		#ENDIF
	}
	else{
		//// partition

		/// access the master boot sector physical address 0 at offset 1BE
		if (read_BLOCK(0x000001BE,buff)==false) return(13);
		#if MMC_INIT_TRACE
			for(i=0;i<32;i++)printf("%2X ",buff[i]);
		#ENDIF
		// start_lsec is address of the partion boot sector
		start_lsec=make32(buff[11],buff[10],buff[9],buff[8]);
		#if MMC_INIT_TRACE
			printf("\n\r boot sector= %lu",start_lsec);
		#ENDIF
		if (read_BLOCK(start_lsec*512,buff)==false) return(14);
	}

	bytes_per_sector=make16(buff[12],buff[11]);
	if(bytes_per_sector!=512) return(15);
	sec_per_cluster=buff[13];
	cluster_size_bytes=(int16)sec_per_cluster*bytes_per_sector;


	sec_resv=make16(buff[15],buff[14]);

	root_dir_entries=make16(buff[18],buff[17]);// number of 32 byte tiles

	total_sectors=make16(buff[20],buff[19]);

	sec_for_FAT=make16(buff[23],buff[22]);
	//branch to file directory
	fat1_address=(start_lsec+sec_resv)*bytes_per_sector;
	fat2_address=fat1_address+bytes_per_sector*sec_for_FAT;
	root_dir_address=(sec_for_FAT*2+start_lsec+sec_resv)*bytes_per_sector;
	data_area_address=root_dir_address+root_dir_entries*32;
	///// check for FAT16
	root_dir_sectors=root_dir_entries>>4;

	sec_for_data=total_sectors - sec_resv -sec_for_fat*2 -root_dir_sectors;

	count_of_clusters=sec_for_data/sec_per_cluster;

	if (count_of_clusters <4085 || count_of_clusters>65525) return(17);

	winhex_adj=fat1_address-bytes_per_sector;

	#if MMC_INIT_TRACE

		printf("Files:/n/r");
		for(i=0;i<MAX_FILES;i++){
			printf("/n/r",file[i].name[i]);
		}

	#ENDIF

	return(0);
}


#separate
int get_CID(char s){
	int i,r1;
	r1=mmc_cmd(0x4A,0x00000000,16,0x00,0x99);


	if (r1==0x00) goto get_token ; // we can read data payload

	return(false);
	get_token:
	for(i=0;i<16;i++)if (SPI_READ(0xFF)==0xFE) goto read_CID; // read token $FE
	return(false);
	read_CID:
	//for (i=0;i<18;i++) s[i]=SPI_READ(0xFF);

	return(true);
}


#separate
int get_CSD(char s){
	int i,r1;
	r1=mmc_cmd(0x4A,0x00000000,16,0x00,0x99);


	if (r1==0x00) goto get_token ; // we can read data payload

	return(false);
	get_token:
	for(i=0;i<16;i++)if (SPI_READ(0xFF)==0xFE) goto read_CSD; // read token $FE
	return(false);
	read_CSD:
	//for(i=0;i<18;i++) s[i]=SPI_READ(0xFF);

	return(true);
}


#separate
int write_BLOCK( int32 address,char *buff,int16 size)
{
	/// low level write ....MMC restriction is that exactly 512 bytes must be written
	/// so a 512 byte section is read in starting at address the first (size) bytes
	/// are over written with the new data and the updated 512 bytes written back
	/// the starting address of the block that contains the requeseted address
	///
	/// the data may span a block if so it is split and two writes are done
	/// so as to maitain MMC 512 write boundary restrictions

	int r1,a,b,c,d;
	int16 i,blk_offset,bytes_posted;
	char tmp_buff[512];
	int32 block_address;

	#if MMC_WRITE_BLOCK_TRACE
		printf("addr=%lu",address);
	#endif

	a=make8(address,3);
	b=make8(address,2);
	c=make8(address,1);
	c=c & 0b11111110;
	d=0;
	block_address=make32(a,b,c,d); //// address int divided by 512
	#if MMC_WRITE_BLOCK_TRACE
		printf("wb>> size=%lu payload=",size);
		for(i=0;i<size;i++)printf("%c",buff[i]);
	#endif

	/// first set up the block size to 512
	if(set_BLOCKLEN((int32)512)==false) return(false); // sets global block_size

	if(block_size!=512) return(false);
	bytes_posted=0; /// no data updated yet

	////////////////////////////////////////////////
	next_block: /// loop back here for second block
	////////////////////////////////////////////////
	#if MMC_WRITE_BLOCK_TRACE
		printf("\n\r blk addr=%lu \n\r",block_address);
	#endif

	if((block_address < data_area_address) && MMC_dir_protected) return(false);

	MMC_dir_protected=true;
	#if MMC_WRITE_BLOCK_TRACE
		printf("read blk");
	#endif
	/// first read in the existing block
	if(read_block(block_address,tmp_buff)==false) return(false) ;



	/// now update the block with new data
	blk_offset=(address - block_address); /// offset within the block
	#if MMC_WRITE_BLOCK_TRACE
		printf("blk_offset=%lu size=%lu",blk_offset,size);
	#endif

	if( blk_offset + size > 512 ){
		// data spans the block so write to end of block first

		#if MMC_WRITE_BLOCK_TRACE
			//// original data
			printf("\n\r spans wb=");
			for(i=blk_offset;i<512;i++)printf("%c",tmp_buff[i]);
		#endif

		for (i=blk_offset;i < 512;i++)tmp_buff[i]=buff[i-blk_offset];

		#if MMC_WRITE_BLOCK_TRACE
			/// updated data
			printf("\n\r spans wb*=");
			for(i=blk_offset;i<512;i++)printf("%c",tmp_buff[i]);
		#endif

		bytes_posted=512-blk_offset; /// wrote from offset to end of block

		#if MMC_WRITE_BLOCK_TRACE
			printf("\n\r posted=%lu",bytes_posted);
		#endif

	}
	else{
		//original or remaining spanned block data fits in next block or original block

		#if MMC_WRITE_BLOCK_TRACE
			printf(" blk offset=%lu",blk_offset);
			/// original data
			printf("\n\r wb=");
			for(i=blk_offset;i<blk_offset+size;i++)printf("%c",tmp_buff[i]);
		#endif

		for (i=blk_offset;i<blk_offset+ size;i++)tmp_buff[i]=buff[bytes_posted+i-blk_offset];

		#if MMC_WRITE_BLOCK_TRACE
			/// updated data
			printf("\n\r wb*=");
			for(i=blk_offset;i<blk_offset+size;i++)printf("%c",tmp_buff[i]);
		#endif

		bytes_posted=size;

		#if MMC_WRITE_BLOCK_TRACE
			printf("\n\r posted=%lu",bytes_posted);
		#endif

	}

	///////////////////////////////////
	/////////// write out the block
	//////////////////////////////////
	#if MMC_WRITE_BLOCK_TRACE
		printf("wb>> writing block %lu",block_address);
	#endif
	r1=mmc_cmd(0x58,block_address,16,0x00,0x40);


	if (r1==0x00) goto send_token ; // we can send data payload
	if (r1==0x40) goto invalid;


	return(false);
invalid:
	printf("\n\r write block err %2X\n\r",r1);
	return(false);
	send_token:
	SPI_READ(0xFE);

	for (i=0;i < 512;i++) {

		SPI_READ(tmp_buff[i]); /// send payload
	}


	SPI_READ(0xFF); // send dummy chcksum
	SPI_READ(0xFF);
	r1=SPI_READ(0xFF);
	for( i=0;i<0x0fff;i++) {
		r1=SPI_READ(0xFF);// digest prior operation
		if (r1!=0x00) break;
	}

	if(size > bytes_posted){
		/// data spanned block so we need to upadte next block as well
		size=size-bytes_posted;
		block_address=block_address+512;/// advance a block

		address=address+bytes_posted; /// move address ptr forward

		goto next_block;
	}


	return(true);
}


#separate
void dump_block(){
	int in_buff[12],c,i,j;
	int32 read_address;
	char buff[MMC_BUFF_SIZE+1];
	for(i=0;i<12;i++)in_buff[i]=0;
	printf("\n\r Input Start address:");
	j=0;
	do {
		c=getc();
		in_buff[j++]=c;
		putc(c);
	}
	while(c!=13);
	in_buff[j-1]=0;

	read_address=atoint32(in_buff);
	if (read_BLOCK(read_address,buff)==true){
		printf(" BLOCK\n\r");
		for(j=0;j<MMC_BUFF_SIZE;j=j+8){
			printf("%4LX ",read_address+j);
			for(i=0;i<8;i++)printf(" %2X",buff[i+j]);
			printf("\n\r");

		}
	}
	else printf("\n\r read_BLOCK failed");

}


#separate
int32 cluster_addr(int fnbr,int mode){
	int32 address;
	char buff[2]; //// buffer for 2 byte ptrs
	///// returns the physical address in the data area of the data pointed to by either the
	///// root cluster or the next cluster in the chain
	/////
	///// if ROOT_CLUSTER is called then this routine returns the address of the first cluster
	///// and assigns this_cluster_ptr and next_cluster_ptr
	/////
	///// if NEXT_CLUSTER is called then this routine returns the address of the next cluster
	///// using the existing next_cluster ptr number
	///// and moves the existing next_cluster ptr number into this_cluster
	///// and assigns the new next cluster ptr number (FFFF) if at the end of chain
	///// if NEXT_CLUSTER is called and the next_cluster_ptr number is FFFF
	///// an address of FFFFFFFF is returned

	///// uses the globals cluster_size_bytes,data_area_address
	//// file struct has the base=root cluster ptr, current=this cluster ptr ,next =cluster chain ptr

	//// !!!! a call with NEXT_cluster must have a valid next_cluster_ptr value
	//// !!!! a call to THIS CLUSTER must have a valid this_cluster_ptr

	//// !!!! Fopen logic considers the cluster prt in the directory tile
	//// to be a next=next_cluster_ptr so NEXT_CLUSTER is used to calc the physical address
	//// of the first root cluster this also assigns the current=this_cluster_ptr
	/// and fetches the next cluster prt
	////
	#IF MMC_CLUSTER_TRACE // if true prints to serial port
		printf("\n\r cluster addr>> next_cluster_ptr= %lu this_cluster=%lu \r\n",file[fnbr].next_cluster_ptr,file[fnbr].this_cluster_ptr);
	#ENDIF
	if (mode==NEXT_CLUSTER){
		///access the next cluster in the chain
		/// requires a valid this_cluster_ptr number and a valid next_cluster_ptr number

		if(file[fnbr].next_cluster_ptr==0xFFFF){
			#IF MMC_CLUSTER_TRACE // if true prints to serial port
				printf("last cluster");
			#ENDIF
			address=0XFFFFFFFF;
		}
		else{
			if(set_BLOCKLEN((int32)2)==false) return(35); /// set up to read 2 bytes
			if(read_BLOCK(fat1_address+(file[fnbr].next_cluster_ptr)*2,buff)==false) return(33);
			file[fnbr].this_cluster_ptr=file[fnbr].next_cluster_ptr; // update current with prev next in chain
			file[fnbr].next_cluster_ptr=make16(buff[1],buff[0]); /// update next in chain

			address=((int32)file[fnbr].this_cluster_ptr-(int32)2)*(int32)cluster_size_bytes+
			data_area_address;

		}
	}
	if (mode==ROOT_CLUSTER){
		//// root_cluster_ptr was assigned from the file tile in fopen
		file[fnbr].this_cluster_ptr=file[fnbr].root_cluster_ptr;
		if(set_BLOCKLEN((int32)2)==false) return(35); /// set up to read 2 bytes
		if(read_BLOCK(fat1_address+(file[fnbr].this_cluster_ptr)*2,buff)==false) return(33);
		file[fnbr].next_cluster_ptr=make16(buff[1],buff[0]); /// update next in chain
		address=((int32)file[fnbr].this_cluster_ptr-(int32)2)*(int32)cluster_size_bytes+
		data_area_address;

	}

	// printf("clust addr call fnbr=%u blk_size=%lu",fnbr,file[fnbr].rec_size);

	if(set_BLOCKLEN(file[fnbr].rec_size)==false) return(37); /// reset to original rec_size

	#IF MMC_CLUSTER_TRACE // if true prints to serial port
		printf("\n\r cluster addr>> next_cluster_ptr*= %lu this_cluster*=%lu \r\n",file[fnbr].next_cluster_ptr,file[fnbr].this_cluster_ptr);
	#ENDIF return(address);
}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////// OPEN FILE ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
#separate
int open_file(int fnbr,char *fname,int16 rec_length){
	int32 dir_addr_ptr;
	int16 bytes_read;
	int i,ptr1,ptr2,lnf_tiles,j;
	char file_name[12];
	int attribute,directory,archive;
	char tmp;
	char buff[32]; //// buffer for 32 byte tiles

	int level; /// level in the directory structure 0 is top
	/////// directory is searched and if file is found

	//////
	////// init_MMC(tries) must be called first
	////// uses globals root_dir_address

	//start by searching the root directory for folder or file

	/// assign an inital next_cluster_ptr in the root directory
	file[fnbr].next_cluster_ptr=0;
	file[fnbr].this_cluster_ptr=0;

	dir_addr_ptr=root_dir_address;
	file_name[11]=0;
	level=0;
	ptr1=0;
	ptr2=0;
	bytes_read=0; //// byte read so far in this cluster

	read_directory:
	/// extract the directory levels(folders)
	while ((fname[ptr2]!='/') && (fname[ptr2]!='\\') && (fname[ptr2]!='\0') && (fname[ptr2]!='.') ){
		// a dos directory (folder) name can not exceed 8 chars
		if ((ptr2-ptr1)>7) return (20);
		ptr2++;
	}
	#IF MMC_OPEN_TRACE
		printf("\n\r fopen ptr1=%u ptr2=%u ",ptr1,ptr2);
	#ENDIF

	if (ptr2==0){ ptr2=1;ptr1=1;goto read_directory;} /// skip a leading '/' or '\'
	if ((ptr2-ptr1)==0) return (21);

	// ptr1 is the chars processed so far
	// ptr2 is the position of '/' or '\' or '.' or '\0'
	// prepare the file or directory name fomat is cccccccceee
	// c is a valid letter or blank eee is extension or blank
	// a directory name is 'cccccccc ' a file 'cccccccceee' always 11 chars
	for(i=0;i<11;i++)file_name[i]=32;//blank
	file_name[11]=0;
	i=0;
	while(ptr1<ptr2){
		// extract the name

		tmp=fname[ptr1];
		tmp=TOUPPER(tmp);
		file_name[i]=tmp;
		ptr1++;i++;
	}
	if(fname[ptr2]=='.'){
		// extract the extension
		i=8;
		while((fname[ptr1]!='\0') && (i<12)){
			ptr1++;
			tmp=fname[ptr1];
			file_name[i]=TOUPPER(tmp);
			i++;
		}
	}
	ptr1++;
	ptr2=ptr1; // advance over the '\' or '/' so next pass starts correctly
	if (block_size!=(int32)32){
		if(set_BLOCKLEN((int32)32)==false) return(17); /// tiles are 32 bytes
	}
	if (read_BLOCK(dir_addr_ptr,buff)==false) return(10);

	// decode the FAT16 entries
	// a tile is 32 bytes
	// std dos files take one tile
	// a long file name has multiple tiles
	// starting with the last down to the first and
	// then a std dos tile is found
	// byte 11 is 0x0f for LNF tiles and 0x00 for std
	// we skip the LNF and goto STD tile

	tile_decode:
	lnf_tiles=0;
	if (buff[0]==0xE5) goto next_tile; ///0xE5 is the deleted file flag
	if (buff[0]==0x00){
		printf("\n\r file err [%s] not found \n\r",file_name);
		return(11); /// file not found
	}
	if (buff[11]==0x0F){
		/// get number of LNF tiles
		lnf_tiles=buff[0] & 0b00111111;
		bytes_read=bytes_read+lnf_tiles*32;
		if(bytes_read>cluster_size_bytes){
			// compute next cluster address next_cluster_ptr must be valid
			// assigns this_cluster_ptr

			dir_addr_ptr=cluster_addr(fnbr,NEXT_CLUSTER);
			if (dir_addr_ptr==0xFFFFFF) return (22);
			bytes_read=bytes_read-cluster_size_bytes;
			dir_addr_ptr=dir_addr_ptr+bytes_read;
		}
		else{
			dir_addr_ptr=dir_addr_ptr+lnf_tiles*32;
		}

		//advance over the lnf tiles
		/// test to see if we need next cluster in chain
		if (read_BLOCK(dir_addr_ptr,buff)==false) return(31);
		/// !!! may read into next sector
	}


	/// check out the standard DOS tile
	#IF MMC_OPEN_TRACE
		printf("\n\r fname[%s] level=%u \n\r",file_name,level);
		for (j=0;j<11;j++)printf("%c",buff[j]);
	#ENDIF
	if(strncmp(buff,file_name, 11)==0){ ///8.3 file name ex "FILE EXT" "FOLDER "
		// we have a file type or a sub directory(folder)
		// so we get the starting cluster number
		attribute=buff[11];

		file[fnbr].root_cluster_ptr=make16(buff[27],buff[26]);/// assign initial cluster ptr
		/// if it is not a directory
		/// it points to the begining of the file
		/// cluster chain



		if ((attribute & 0b00010000)>0)directory=true;
		else directory=false;
		if ((attribute & 0b00100000)>0 || attribute==0){
			archive=true; //// we have our file

			file[fnbr].size=make32(buff[31],buff[30],buff[29],buff[28]);
			file[fnbr].dir_addr_ptr=dir_addr_ptr; ///save address of this files tile
			/// assign global value
		}
		else archive=false;



		goto match_found;
		// goto fill_table; // we have a match
	}
	next_tile:
	bytes_read=bytes_read+32;
	if(bytes_read > cluster_size_bytes){
		/// requires a valid next=next_cluster_ptr
		// compute next cluster address and assign this cluster
		dir_addr_ptr=cluster_addr(fnbr,NEXT_CLUSTER);
		if (dir_addr_ptr==0xFFFFFF) return (23);
		bytes_read=bytes_read-cluster_size_bytes;
		dir_addr_ptr=dir_addr_ptr+bytes_read;
	}
	else{
		dir_addr_ptr=dir_addr_ptr+32;
	}


	dir_addr_ptr=dir_addr_ptr+32;

	if (read_BLOCK(dir_addr_ptr,buff)==false) return(32);
	goto tile_decode;

	match_found:
	///// if we have a sub directory we need to cycle down a level
	if (directory==true) {
		// compute the sub directory address
		// compute this cluster address this_cluster_ptr must be valid
		dir_addr_ptr=cluster_addr(fnbr,ROOT_CLUSTER); /// set physical addr of starting cluster
		#IF MMC_OPEN_TRACE
			printf("\n\r next_cluster_ptr=%lu \n\r ",file[fnbr].next_cluster_ptr);
		#ENDIF
		//printf("\n\r dir_addr_ptr=%lu",dir_addr_ptr);
		// dir_addr_ptr=((int32)cluster_table[0]-(int32)2)*(int32)cluster_size_bytes+
		// data_area_address;
		level++;
		goto read_directory;
	}


	// note record length must divide into 512 to align properly
	if (rec_length<2) return(12);



	/// get the initial file_addr_ptr

	file[fnbr].addr_ptr=cluster_addr(fnbr,ROOT_CLUSTER);
	file[fnbr].offset=0; //init bytes read from beginning of open file
	file[fnbr].cluster_offset=0; //init bytes read to beginning of the current cluster
	file[fnbr].rec_size=(int32)rec_length; /// assign file record size
	#IF MMC_OPEN_TRACE
		printf("root_cluster=%lu \n\r",file[fnbr].root_cluster_ptr);
	#ENDIF

	//printf("\n\r fopen %u rec size=%lu",fnbr,file[fnbr].rec_size);

	if(set_BLOCKLEN(file[fnbr].rec_size)==false) return(13);

	return(0);
}

//////////////////////////////////////////////////////////////////////////////////
////////////////////////////// FILE READ ///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
#separate
int file_read(int8 fnbr,char *buff){
	int32 address;
	int32 nxt_cluster;
	//// MMC allows a read to start and stop at any address but this file system
	//// imposes a record size restriction the record size must divide into the
	/// 512 block to allow writing of the records
	/// rec_size must align with cluster boundary 2048 ...must be a divisor of 2048
	/// find the cluster containing the offset
	/// buff must be at least the size of the recordsize requested in the File open

	//printf("foffset=%lu coffset=%lu ",file[fnbr].offset,file[fnbr].cluster_offset);////$$$$

	if ( file[fnbr].offset>=file[fnbr].size) return(10); /// already beyond eof

	if ( file[fnbr].offset + (int32) file[fnbr].rec_size > file[fnbr].cluster_offset + (int32) cluster_size_bytes){
		#IF MMC_READ_TRACE
			printf("adv to next cluster");
		#ENDIF
		/// need to advance to the next cluster
		nxt_cluster=cluster_addr(fnbr,NEXT_CLUSTER);
		if ( nxt_cluster!=0XFFFFFFFF) file[fnbr].addr_ptr=nxt_cluster;
		else return(11); /// last cluster in file reached

		file[fnbr].cluster_offset=file[fnbr].cluster_offset+(int32)cluster_size_bytes; //foffset is the byte offset within the file
		//that file_addr_ptr points to
	}
	address=file[fnbr].addr_ptr+file[fnbr].offset-file[fnbr].cluster_offset;
	#IF MMC_READ_TRACE
		//printf("\n\r offset=%lu",cluster_offset);
		printf("\n\r data_area_address=%lu",address);
		printf("\n\r cluster_size_bytes=%lu",cluster_size_bytes);

		//printf("\n\r file_addr_ptr=%lu",file_addr_ptr);
	#ENDIF

	if (read_BLOCK(address,buff)==false)return(12); /// read block into buff

	if ( file[fnbr].offset+file[fnbr].rec_size< file[fnbr].size ) file[fnbr].offset=file[fnbr].offset+file[fnbr].rec_size;
	else{ /// end of file
		#IF MMC_READ_TRACE
			printf("eof size=%lu",file[fnbr].size);
		#ENDIF
		buff[ file[fnbr].size-file[fnbr].offset]=0; /// short record
		file[fnbr].offset=file[fnbr].size;
		return(255); //eof
	}
	return(0);
}


//////////////////////////////////////////////////////////////////////////////////
////////////////////////////// WRITE FILE /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#separate
int file_write(int8 fnbr,int *buff){
	//// buff size must be at least the recordsize requested in File open
	//// the record is updated only chars beyond rec_size are ignored
	/// set up for write
	/// A MMC write is restricted it must be for a block and allign on block boundaries
	/// blocklen must be exactly 512 and start address must be the begining of a
	/// sector
	/// the buff could potentially span a sector and or span a block(512) boundary
	/// ex there could be 1byte left in a block and 1 byte lect in a sector
	// if the block is the last block in the sector
	/// worst case we could write to two blocks and need a new sector
	int32 address,nxt_cluster;

	int16 in_cluster_size,out_cluster_size;
	int8 appending_flag;
	appending_flag=0;
	if (file[fnbr].offset + file[fnbr].rec_size>=file[fnbr].size) appending_flag=1;


	/// find the cluster containing the offset
	if ( file[fnbr].offset+file[fnbr].rec_size>=file[fnbr].cluster_offset + cluster_size_bytes){
		#IF MMC_WRITE_TRACE
			printf("spanning cluster \n\r");
		#ENDIF
		/// spans the current cluster so we split the write
		in_cluster_size=file[fnbr].cluster_offset+cluster_size_bytes-file[fnbr].offset;
		/// bytes from start of file to end of this cluste- bytes into the file
		out_cluster_size=file[fnbr].rec_size - in_cluster_size;
		#IF MMC_WRITE_TRACE
			printf("write>> spanning cluster inside=%lu outside=%lu \n\r",in_cluster_size,out_cluster_size);
		#ENDIF
		address=file[fnbr].addr_ptr+file[fnbr].offset - file[fnbr].cluster_offset;
		// physical address=
		// physical address of the cluster +offset from begining of file
		// - offset from the begining of file for the byte at the begining of the cluster
		#IF MMC_WRITE_TRACE
			printf("write file>>cluster=%lu in clstr addr=%lu",file[fnbr].this_cluster_ptr,address);
		#ENDIF
		//// address=physical offset of this cluster +bytes into this cluster
		if(write_BLOCK(address,buff,in_cluster_size)==false)return(81); //// write first chunk



		/// allocate the next cluster
		nxt_cluster=cluster_addr(fnbr,NEXT_CLUSTER); ///physical address of file data that the
		/// specific cluster indexes
		#IF MMC_WRITE_TRACE
			printf("nxt_cluster=%lu",nxt_cluster);
		#ENDIF

		if ( nxt_cluster==0xFFFFFFFF){
			#IF MMC_WRITE_TRACE
				printf("updating FAT");
			#ENDIF
			//// FAT2 is an identical copy of FAT1
			file_new_cluster(fnbr,1); /// a new cluster is allocated in FAT1
			file_new_cluster(fnbr,2); /// a new cluster is allocated in FAT2
			nxt_cluster=cluster_addr(fnbr,NEXT_CLUSTER); ///physical address of file data that the
			#IF MMC_WRITE_TRACE
				printf("\n\r write>>nxt_cluster addr=%lu this clstr=%lu next=%lu",nxt_cluster,file[fnbr].this_cluster_ptr,file[fnbr].next_cluster_ptr); /// specific cluster indexes
			#ENDIF
		}

		file[fnbr].addr_ptr =nxt_cluster;
		file[fnbr].cluster_offset=file[fnbr].cluster_offset + cluster_size_bytes; //foffset is the byte offset within the file
		//that file_addr_ptr points to
		address=file[fnbr].addr_ptr + file[fnbr].offset - file[fnbr].cluster_offset + in_cluster_size;
		#IF MMC_WRITE_TRACE
			printf("out addr=%lu,out size=%lu",address,out_cluster_size);
		#ENDIF
		if(write_BLOCK(address,&buff[in_cluster_size],out_cluster_size)==false)return(82); /// write block pads with 0x00 to end of sector
	}// end of spanned cluster
	else{
		/// within the current cluster
		address=file[fnbr].addr_ptr+file[fnbr].offset - file[fnbr].cluster_offset;



		if(write_BLOCK(address,buff,file[fnbr].rec_size)==false)return(84); /// write block pads with 0x00 to end of sector

	}
	if(appending_flag==1) {
		/// if appended we need to up date the file size
		file[fnbr].size=file[fnbr].size + file[fnbr].rec_size; /// add one record
		address=file[fnbr].dir_addr_ptr+28; /// file size is offset 28 in tiles
		#IF MMC_WRITE_TRACE
			printf("new file size=%lu",file[fnbr].size);
		#ENDIF
		buff[0]=make8(file[fnbr].size,0);
		buff[1]=make8(file[fnbr].size,1);
		buff[2]=make8(file[fnbr].size,2);
		buff[3]=make8(file[fnbr].size,3);
		MMC_dir_protected=false;
		if(write_BLOCK(address,buff,4)==false)return(85);
	}
	if(set_BLOCKLEN(file[fnbr].rec_size)==false) return(86); /// reset to original rec_size
	return(0);
}


#separate
int file_set(int fnbr,int32 offset){
	/// file open sets the offset to the begining offset=0
	/// this sets the offset within the file ...offset of 0 is a reset

	if(offset>=file[fnbr].size) return(71);

	file[fnbr].offset=offset; //// overwrite the existing offset
	file[fnbr].next_cluster_ptr=file[fnbr].root_cluster_ptr; /// set current ptr to beginning
	file[fnbr].cluster_offset=0;
	// move the cluster to the one containing the offset

	while ( offset>cluster_size_bytes ){

		file[fnbr].addr_ptr=cluster_addr(fnbr,NEXT_CLUSTER);
		file[fnbr].cluster_offset+=cluster_size_bytes; //foffset is the byte offset within the file
		if (offset-cluster_size_bytes >0) offset= offset - cluster_size_bytes;

	}
	return(0);
}


#separate
int file_new_cluster(int8 fnbr,int8 mode){ ///////////// this does identical writes to either the FAT1 and FAT2 sectors
	int16 eof_cluster;

	char buff[2],tmp_buff[2];
	int32 address;
	int32 fat_address;
	int16 slot;
	/// an unused cluster has the value 0x0000 as its next cluster ptr
	/// a used cluster has either 0xFFFF meaning last in chain
	/// or a valid cluster displacement in the FAT1 amd FAT2 area
	/// to append a cluster the 0XFFFF needs to be replaced by the appended
	/// cluster location and the appended locations data (next ptr) needs to be set to 0XFFFF



	eof_cluster=file[fnbr].this_cluster_ptr;
	#IF MMC_NEW_CLUSTER
		printf("the cluster with eof (FFFF)=%lu \n\r",eof_cluster);
	#ENDIF

	slot=0;
	if(set_BLOCKLEN((int32)2)==false)return(false); // force blocklen to 2
	/// use global address of FAT1 assigned by INIT
	if (mode==2)fat_address=fat2_address;
	else fat_address=fat1_address;
	address=fat_address;

	#IF MMC_NEW_CLUSTER
		printf("mode=%u FAT addr=%lu \n\r",mode,address);
	#ENDIF

	do{
		if(read_block(address,buff)==false) return(false) ;
		slot=slot+1;
		address=address+2;
		//printf(" slot %lu =%2x %2x",slot,buff[0],buff[1]);
	}
	while (buff[0]!=0 || buff[1]!=0);

	address=address-2; // correct for over step
	slot=slot-1;


	#IF MMC_NEW_CLUSTER
		printf("slot=%lu address=%lu",slot,address);
	#ENDIF

	/// found an unused cluster
	tmp_buff[0]=0xFF;tmp_buff[1]=0xFF; /// stamp it as last
	MMC_dir_protected=false; /// allow writes to the protected areas
	if(write_block(address,tmp_buff,2)==false ) return(false);

	/////////////////////////////////////////////
	/// update prev cluster with 0xFFFF in it
	tmp_buff[1]=make8(slot,1);
	tmp_buff[0]=make8(slot,0);
	if (mode==1){
		//// update the file info
		file[fnbr].next_cluster_ptr=slot;
		#IF MMC_NEW_CLUSTER
			printf("cluster %lu was updated to point to %lu",file[fnbr].this_cluster_ptr,file[fnbr].next_cluster_ptr);
		#ENDIF
	}
	/// compute physical address of the current cluster
	MMC_dir_protected=false; /// allow writes to the protected areas
	if(write_BLOCK(fat_address+(file[fnbr].this_cluster_ptr)*2,tmp_buff,2)==false) return(33);
	if(set_BLOCKLEN((int32)file[fnbr].rec_size)==false)return(false); // reset blocklen

	return(true);
}


signed int strncmp(char *s1, char *s2, int n){
	for (; n > 0; s1++, s2++, n--){
		if (*s1 != *s2) return((*s1 <*s2) ? -1: 1);
		else if (*s1 == '\0') return(0);
	}
	return(0);
}
