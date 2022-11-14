char *itoa(int value, char *string, int radix);
char *ltoa(long value, char *string, int radix);

// #define DEBUGSHM  // debug shared memory for plot

#define OUAL_MAX_DEFINES 150
#define OUAL_MAX_DEFINE_LENGTH 1048576 
#define OUAL_MAX_NAME_LENGTH 16
#define OUAL_MAX_VECTORS 150
#define OUAL_MAX_CMND_LEN 132
#define OUAL_BUFFERSIZE 10240
/*

	type: 'd' = data array

	The far pointer will support array definitions < 65536 bytes
	long.  If longer arrays are needed they must be huge (much slower).

*/
/* new addition 5-17-96 for RUMP file support */
/*-------------------------------------------------------------------------*/
/* int rumpread()															   */
/* Notes: The pointer rump_define must be valid.  spectra->counts should be    */
/*        to a (void *) NULL pointer so that rd_spectra will allocate      */
/*        space for the data and set the pointer as necessary.             */
/*	Read a RUMP(.rbs) file or compressed data format (.cdf)				   */
/*  RUMP file is a type 5                                                  */
/*	CDF file is any other type.											   */
/*-------------------------------------------------------------------------*/
extern  int rumpread();
/*-------------------------------------------------------------------------*/
/* int rumpwrite()															   */
/* Notes: All components of the SPECTRA_HEADER must be defined before this */
/*        call.  The format and record order corresponds to the historical */
/*        values of the RUMP program for simple RBS files.	           	   */
/*	Write a RUMP(.rbs) file or compressed data format (.cdf)			   */
/*  RUMP file is a type 5                                                  */
/*	CDF file is any other type.											   */
/*-------------------------------------------------------------------------*/
extern	int  rumpwrite();

extern int saveCompressed();				/* Compressed data write */
extern int writeCompressed(int, unsigned *);	/* RUMP file write */

/*
extern struct rump_header {
	char filename[80];                  // Filename
	char date[80];                      // Date spectrum collected
	char ltct[80];                      // Live time/Clock time information
	char id[80];                        // Identifier string
	float e0;                           // Incident energy            (MeV)
	int   zbeam;                        // Atomic Z of incident
	float mbeam;                        // Atomic mass of incident    (amu)
	int   cbeam;                        // Charge state of beam
	float q;                            // Total accumulated charge    (uC)
	float current;                      // Average beam current        (nA)
	float scale1,scale2;                // Conversion MCA chan # -> keV
	float first;                        // Channel number of first data pt
	float fwhm;                         // Detector resolution        (keV)
	int   geom;                         // Geometry identifier
	float phi,theta,psi;                // Scattering angles      (degrees)
	float omega;                        // Detector solid angle       (mSr)
	float corr;                         // Random correction factor
};
*/

extern struct databuffer
{
        char dav;
        unsigned nevents;
        unsigned device[OUAL_BUFFERSIZE];
        unsigned channel[OUAL_BUFFERSIZE];
        unsigned data[OUAL_BUFFERSIZE];
        unsigned abort;
	char rpc_command[80];
	unsigned rpc_flag;
}
         oual_buff;

extern struct plotbuffer
{
        unsigned start;
        unsigned stop;
        unsigned long savdat;
        unsigned long savtim;
        int type;
        int inuse;
        unsigned refid;
        unsigned data [OUAL_MAX_DEFINE_LENGTH];
        } oual_plot;

extern int pid;
extern char *shmptr;
extern char *shmpltptr;
extern struct databuffer *databufferptr;
extern struct plotbuffer *plotbufferptr;

extern struct
{
	unsigned type;
	char nbytes;
	unsigned  *address;
	long length;
	char name[OUAL_MAX_NAME_LENGTH];
	unsigned conversion_gain;
	unsigned shift;
	unsigned digital_offset;
	unsigned lld;
	unsigned uld;
/* The next three lines added July 93 - ceb */
	float slope;
	float intcept;
	int ndat;
/* next two lines added to support prescale  dec */
	int prescale_init;
	int prescale;
/* next line added to support rump file definition jck */
	struct rump_header * rump_define;

} oual_define[OUAL_MAX_DEFINES];


extern unsigned oual_vector[];
extern char oual_gated_vector[];

extern int oual_ndefines;
extern int oual_nvectors;
extern int oual_disk;
extern int oual_tag;
extern int oual_file_index;
extern int oual_retcode;
extern int oual_sofint;
extern int oual_intmask;
extern int oual_plot_flag;
extern int oual_terminate;
extern int oual_cmnd_nmbr;
extern unsigned oual_counter;

/* Common command line buffer */

extern char oual_cmnd[133];			/* last token read by tokenizer */
extern int oual_last_token;			/* last result returned by gettoken() */

#define OUAL_MAX_CMND_LEN 132

typedef enum { T_WORD, T_BAR, T_AMP, T_SEMI, T_GT, T_GTGT, T_LT,
			   T_NL, T_EOF, T_INTEGER, T_FLOAT} TOKEN;

extern FILE *oual_iunit;
extern FILE *oual_ounit;

extern unsigned oual_rawdata, oual_rawroute, oual_adcnumber, oual_adcdata, oual_router, oual_anal;

extern int oual_nest_level;
extern char oual_nest_name[9];
extern int oual_nrgates, oual_maxrgates;

extern unsigned oual_gate[],oual_rgate[];
extern unsigned oual_ngates, oual_t0;
extern unsigned oual_verbose;
extern long oual_return1;
extern int oual_prescale, oual_prescale_init;
