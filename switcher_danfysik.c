
/*----------------------------------------------------------------------------
 *  Copyright (c) 1991, 1992, 1993, 1994   Ohio University
 *                                         Accelerator Laboratory
 *
 *  This software was partially developed under a United States Government
 *  license described in the Notice file included as part of this distribution
 * 
 *    Ohio University Accelerator Laboratory, Athens, Ohio  45701
 * 
 *    carter@oual3.phy.ohiou.edu   Tel: (614) 593-1984  Fax: (614) 593-1436 
 *    odonnell@oual3.phy.ohiou.edu Tel: (614) 593-1977  Fax: (614) 593-1436 
 *----------------------------------------------------------------------------
 *
 * Authors:
 *
 *	Don Carter
 *      John O'Donnell
 *
 *	Ohio University Accelerator Laboratory
 *
 * Original version DAQ created 1991
 * Revision History:
 *
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <varargs.h>

#include <signal.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "../oual.h"
#include "/usr/include/math.h"
  
#include <Xm/DrawingA.h>
#include <Xm/PushB.h>
#include <Xm/Form.h>
#include <Xm/Label.h>

#include <Xm/MainW.h>
#include <Xm/PushBG.h>
#include <Xm/RowColumn.h>
#include <Xm/ScrolledW.h>
#include <Xm/Xm.h>

#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/cursorfont.h>

#include <termios.h>
#include <unistd.h>


// following for rpc command support to HP

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#define PORT        5025
#define HOST        "10.0.0.103"
#define DIRSIZE     8192
// analyzer #define SHUNT_FACTOR 27.78774
#define SHUNT_FACTOR 22.10121
#define MAX_AMPS    250
// analyzer #define XSCALE      8.970
#define XSCALE      11.478
#define YSCALE     16.0
char    dir[DIRSIZE];

struct termios options;
int fd;

int	time=500;
struct sockaddr_in sin;
struct sockaddr_in pin;
struct hostent *hp;

float xval=0.01,yval=6.0,txval,tyval,save_xval=0.01,save_yval=6.0;
float xvalmax=13.7;

//

void Ex1CB();
void Ex2CB();
void Ex3CB();
void Ex4CB();
void ZoomCB();
void UnZoomCB();
void PanlCB();
void PanrCB();
void YupCB();
void YdnCB();
void AutoCB();
void HomeCB();
void RefCB();
void QuitCB();
void RedrawCB();
void ResizeCB();
void xs_wprintf();
void TimeOutCB();
int  readdata();
int  updatedata();
void start_rubber_band();
void end_rubber_band();
void track_rubber_band();
void mouse_motion();
//void focus_change();
void enter_window();
void exit_window();
void forcedexit();

GC	SimpleGC;
Pixmap pixmap;

int CBcount = 0;
int id;
int parent;
int refloops, mousemoved;
int rtype;
int halt_update=0,rubberband=0,highres=0;
int current_x=0,current_y=0,mouse_channel,mouse_value;
int previous_sum=0;
float rate_factor = 1.0;
Dimension new_width, new_height;

String colors[] = {
   "Black", "Grey", "Red", "Brown", "Purple", "Green", "Blue", 
   "Magenta", "Turquoise", "Cyan", "Wheat", "Violet", "LimeGreen",
   "Yellow", "Pink", "White" 
};  

int magnet_lock = 0;
int realtime = 1;
int ouColors = 0;
long ixval;
unsigned long ouZcolor[16];
unsigned long ouBlack,ouWhite,ouGray,ouMagenta,ouCyan,ouYellow,ouBlue,ouRed,ouGreen;

struct dataplot {
	int	data[OUAL_MAX_DEFINE_LENGTH];
	int	disk,tag,start,stop,max,min,istart,istop,autoy;
	int     parent;
	Widget	info;
} plot;

typedef struct {
        int start_x, start_y, last_x, last_y;
        GC gc;
} rubber_band_data;

/* struct plotbuffer {
	unsigned start;
	unsigned stop;
	unsigned long savdat;
	unsigned long savtim;
	int inuse;
        unsigned refid;
	unsigned data[OUAL_MAX_DEFINE_LENGTH];
	} oual_plot;
*/
char *shmpltptr;
struct plotbuffer *plotbufferptr;

	Widget  canvas;
 
main(argc, argv)
	int argc;
	char *argv[];
{
	Widget 	toplevel, frame, infobar, quitbutton, ex1, ex2, ex3, ex4, home;
	Widget  zoom, unzoom, panl, panr, yauto, yup, ydn, ref;
        rubber_band_data data;
	XtAppContext app;
	XSetWindowAttributes	attrs;
	XmString	xmstr;
	Arg	wargs[10];
	extern GC	SimpleGC;
	XGCValues	gcvalues;
	int	n;
 	int  	j;
	unsigned long	valuemask;
	char command[100];

        char xvalstring[21];

	toplevel = XtVaAppInitialize(&app, "Simple", NULL, 0, 
				&argc, argv, NULL, NULL);

	/* XtInitialize is called first to let to digest any command
		line arguments for windows; it will leave anything it
		doesn't want for us. */
//      printf("\nEntering fpdcrefx...");
/*
	printf("Enter disk and tag number to plot:");
	scanf("%d %d",&plot.disk,&plot.tag);
*/
	//id = atoi(argv[2]);
	//parent = atoi(argv[3]);
        //refloops = atoi(argv[4]);
 
	id = 0;
	parent = 0;
	refloops = 0;

        mousemoved = 0;
  	if (readdata(plot.disk,plot.tag,&plot) != 0) {
  		printf("readdata failed\n");
  		exit(0);
  	}

	/* data entered for quad mods */
	plot.start=0;
	plot.stop=1000;

    fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {
        printf("Could not open port /dev/ttyS0\n");
        return 1;
    }

    tcgetattr(fd, &options); //get port options

    cfsetispeed(&options, B9600); //set input baud rate
    cfsetospeed(&options, B9600); //set output baud rate

    options.c_cflag |= (CLOCAL | CREAD); //enable receiver and set local mode

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS; //disable hardware flow control
    options.c_cflag &= ~(ICANON | ECHO | ISIG); //set raw input

    options.c_cflag |= IXOFF; //disable software flow control

    tcsetattr(fd, TCSANOW, &options); //set new port options

    printf("Opened successfully /dev/ttyS0\n");

    sleep(1);

/*
	printf("Readdata completed, disk %d tag %d start %d stop %d\n",
			plot.disk,plot.tag,plot.start,plot.stop);

*/
  
	n = 0;
	XtSetArg(wargs[n], XmNheight, 480); n++;
	XtSetArg(wargs[n], XmNwidth, 640); n++;
        XtSetArg(wargs[n], XmNx    , 640); n++;
        XtSetArg(wargs[n], XmNy    , 480); n++;
/*
	XtSetArg(wargs[n], XmNheight, 500); n++;
        XtSetArg(wargs[n], XmNwidth, 700); n++;
        XtSetArg(wargs[n], XmNx    , 700); n++;
        XtSetArg(wargs[n], XmNy    , 700); n++;
*/
	frame = XtCreateManagedWidget("Frame", xmFormWidgetClass,
				toplevel, wargs, n);

	n = 0;
	XtSetArg(wargs[n], XmNleftAttachment, XmATTACH_FORM); n++;
	XtSetArg(wargs[n], XmNrightAttachment, XmATTACH_FORM); n++;
	XtSetArg(wargs[n], XmNtopAttachment, XmATTACH_FORM); n++;
	XtSetArg(wargs[n], XmNorientation, XmHORIZONTAL); n++;
	infobar = XmCreateRowColumn(frame, "infobar", wargs, n);
	XtManageChild(infobar);

	n = 0;
	xmstr = XmStringCreate("Quit", XmSTRING_DEFAULT_CHARSET);
	XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
	XtSetArg(wargs[n], XmNadjustLast, False); n++;
	XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        //XtSetArg(wargs[n], XmNbackground, 0xf08080); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*255+256*211+155); n++; // burlywood
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
	quitbutton = XmCreatePushButton(infobar, "quitbutton", wargs, n);
	XtAddCallback(quitbutton, XmNarmCallback, QuitCB, 1);
	XtAddCallback(quitbutton, XmNactivateCallback, QuitCB, 2);
	XtAddCallback(quitbutton, XmNdisarmCallback, QuitCB, 3);
	XtManageChild(quitbutton);
/*
        n = 0;
        xmstr = XmStringCreate(argv[5], XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 0x00ee76); n++;
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        ex1 = XmCreatePushButton(infobar, "ex1", wargs, n);
        XtAddCallback(ex1, XmNarmCallback, Ex1CB, 1);
        XtAddCallback(ex1, XmNactivateCallback, Ex1CB, 2);
        XtAddCallback(ex1, XmNdisarmCallback, Ex1CB, 3);
        XtManageChild(ex1);

        n = 0;
        xmstr = XmStringCreate(argv[6], XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 0x00ee76); n++;
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        ex2 = XmCreatePushButton(infobar, "ex2", wargs, n);
        XtAddCallback(ex2, XmNarmCallback, Ex2CB, 1);
        XtAddCallback(ex2, XmNactivateCallback, Ex2CB, 2);
        XtAddCallback(ex2, XmNdisarmCallback, Ex2CB, 3);
        XtManageChild(ex2);

        n = 0;
        xmstr = XmStringCreate(argv[7], XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 0x00ee76); n++;
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        ex3 = XmCreatePushButton(infobar, "ex3", wargs, n);
        XtAddCallback(ex3, XmNarmCallback, Ex3CB, 1);
        XtAddCallback(ex3, XmNactivateCallback, Ex3CB, 2);
        XtAddCallback(ex3, XmNdisarmCallback, Ex3CB, 3);
        XtManageChild(ex3);

        n = 0;
        xmstr = XmStringCreate(argv[8], XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 0x00ee76); n++;
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        ex4 = XmCreatePushButton(infobar, "ex4", wargs, n);
        XtAddCallback(ex4, XmNarmCallback, Ex4CB, 1);
        XtAddCallback(ex4, XmNactivateCallback, Ex4CB, 2);
        XtAddCallback(ex4, XmNdisarmCallback, Ex4CB, 3);
        XtManageChild(ex4);
*/
        n = 0;
        xmstr = XmStringCreate("Grab", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        //XtSetArg(wargs[n], XmNbackground, 0x87cefa); n++;
        //XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*255+256*0+0); n++; // red1
        XtSetArg(wargs[n], XmNforeground, 0xffffff); n++;
        home = XmCreatePushButton(infobar, "home", wargs, n);
        XtAddCallback(home, XmNarmCallback, HomeCB, 1);
        XtAddCallback(home, XmNactivateCallback, HomeCB, 2);
        XtAddCallback(home, XmNdisarmCallback, HomeCB, 3);
        XtManageChild(home);
 /* 
        n = 0;
        xmstr = XmStringCreate("A = B", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 0xffff00); n++;
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        yauto = XmCreatePushButton(infobar, "yauto", wargs, n);
        XtAddCallback(yauto, XmNarmCallback, AutoCB, 1);
        XtAddCallback(yauto, XmNactivateCallback, AutoCB, 2);
        XtAddCallback(yauto, XmNdisarmCallback, AutoCB, 3);
        XtManageChild(yauto);
        n = 0;
	
        xmstr = XmStringCreate("dA = dB", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 0xfafad2); n++;
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        ydn = XmCreatePushButton(infobar, "ydn", wargs, n);
        XtAddCallback(ydn, XmNarmCallback, YdnCB, 1);
        XtAddCallback(ydn, XmNactivateCallback, YdnCB, 2);
        XtAddCallback(ydn, XmNdisarmCallback, YdnCB, 3);
        XtManageChild(ydn);

        n = 0;
        xmstr = XmStringCreate("A OR B", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 0xe0ffff); n++;
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        yup = XmCreatePushButton(infobar, "yup", wargs, n);
        XtAddCallback(yup, XmNarmCallback, YupCB, 1);
        XtAddCallback(yup, XmNactivateCallback, YupCB, 2);
        XtAddCallback(yup, XmNdisarmCallback, YupCB, 3);
        XtManageChild(yup);
*/
        n = 0;
        xmstr = XmStringCreate("Fine", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        //XtSetArg(wargs[n], XmNbackground, 0xffb6c1); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*255+256*236+139); n++; // lightgoldenrod
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        zoom = XmCreatePushButton(infobar, "zoom", wargs, n);
        XtAddCallback(zoom, XmNarmCallback, ZoomCB, 1);
        XtAddCallback(zoom, XmNactivateCallback, ZoomCB, 2);
        XtAddCallback(zoom, XmNdisarmCallback, ZoomCB, 3);
        XtManageChild(zoom);

        n = 0;
        xmstr = XmStringCreate("Coarse", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 0x00ee76); n++;
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        unzoom = XmCreatePushButton(infobar, "unzoom", wargs, n);
        XtAddCallback(unzoom, XmNarmCallback, UnZoomCB, 1);
        XtAddCallback(unzoom, XmNactivateCallback, UnZoomCB, 2);
        XtAddCallback(unzoom, XmNdisarmCallback, UnZoomCB, 3);
        XtManageChild(unzoom);
  
        n = 0;
        xmstr = XmStringCreate("Save", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        //XtSetArg(wargs[n], XmNbackground, 0xffb6c1); n++;
        //XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*255+256*0+255); n++; // magenta
        XtSetArg(wargs[n], XmNforeground, 0xffffff); n++;
        panl = XmCreatePushButton(infobar, "panl", wargs, n);
        XtAddCallback(panl, XmNarmCallback, PanlCB, 1);
        XtAddCallback(panl, XmNactivateCallback, PanlCB, 2);
        XtAddCallback(panl, XmNdisarmCallback, PanlCB, 3);
        XtManageChild(panl);

        n = 0;
        xmstr = XmStringCreate("Restore", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        //XtSetArg(wargs[n], XmNbackground, 0x00ee76); n++;
        //XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*230+256*99+99); n++; //indianred1
        XtSetArg(wargs[n], XmNforeground, 0xffffff); n++;
        panr = XmCreatePushButton(infobar, "panr", wargs, n);
        XtAddCallback(panr, XmNarmCallback, PanrCB, 1);
        XtAddCallback(panr, XmNactivateCallback, PanrCB, 2);
        XtAddCallback(panr, XmNdisarmCallback, PanrCB, 3);
        XtManageChild(panr);
/* 
        n = 0;
        xmstr = XmStringCreate("Ref", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        ref = XmCreatePushButton(infobar, "ref", wargs, n);
        XtAddCallback(ref, XmNarmCallback, RefCB, 1);
        XtAddCallback(ref, XmNactivateCallback, RefCB, 2);
        XtAddCallback(ref, XmNdisarmCallback, RefCB, 3);
        XtManageChild(ref);
*/           
                            
        n = 0;
        xmstr = XmStringCreate("Lock", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*255+256*20+147); n++; //deeppink
        XtSetArg(wargs[n], XmNforeground, 0xffffff); n++;
        ref = XmCreatePushButton(infobar, "ref", wargs, n);
        XtAddCallback(ref, XmNarmCallback, RefCB, 1);
        XtAddCallback(ref, XmNactivateCallback, RefCB, 2);
        XtAddCallback(ref, XmNdisarmCallback, RefCB, 3);
        XtManageChild(ref);    

	n = 0;
	xmstr = XmStringCreate("Label", XmSTRING_DEFAULT_CHARSET);
	XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
	plot.info = XmCreateLabel(infobar, "info", wargs, n);
	XtManageChild(plot.info);
  	
	n = 0;
	XtSetArg(wargs[n], XmNheight, 480); n++;
	XtSetArg(wargs[n], XmNwidth, 640); n++;
	XtSetArg(wargs[n], XmNx, 0); n++;
	XtSetArg(wargs[n], XmNy, 50); n++;
	XtSetArg(wargs[n], XmNborderWidth, 1); n++;
	XtSetArg(wargs[n], XmNleftAttachment, XmATTACH_FORM); n++;
	XtSetArg(wargs[n], XmNrightAttachment, XmATTACH_FORM); n++;
	XtSetArg(wargs[n], XmNbottomAttachment, XmATTACH_FORM); n++;
	XtSetArg(wargs[n], XmNtopAttachment, XmATTACH_WIDGET); n++;
	XtSetArg(wargs[n], XmNtopWidget, infobar); n++;
	canvas = XmCreateDrawingArea(frame, "canvas", wargs, n);
	XtAddCallback(canvas, XmNresizeCallback, ResizeCB, 1);
        XtAddCallback(canvas, XmNexposeCallback, RedrawCB, "plot");

        XtAddEventHandler(canvas, ButtonPressMask, FALSE,
                          start_rubber_band, &data);
        XtAddEventHandler(canvas, ButtonMotionMask, FALSE, 
                          track_rubber_band, &data);
        XtAddEventHandler(canvas, ButtonReleaseMask, FALSE, 
                          end_rubber_band, &data);
        XtAddEventHandler(canvas, PointerMotionMask, FALSE,
                          mouse_motion, &data);
	//XtAddEventHandler(canvas, FocusChangeMask, FALSE,
	//	          focus_change, &data);
	XtAddEventHandler(canvas, EnterWindowMask, FALSE,
			 enter_window, &data);
	XtAddEventHandler(canvas, LeaveWindowMask, FALSE,
			  exit_window, &data);

        signal(SIGKILL,forcedexit);
        signal(SIGHUP ,forcedexit);
        signal(SIGSTOP,forcedexit);
        signal(SIGTERM,forcedexit);
        signal(SIGABRT,forcedexit);
        signal(SIGQUIT,forcedexit);

	XtManageChild(canvas);

	xs_wprintf(plot.info," %s", argv[1]);

	XtRealizeWidget(toplevel);

        XGrabButton(XtDisplay(canvas), AnyButton, AnyModifier,
                    XtWindow(canvas), TRUE,
                    ButtonPressMask | ButtonMotionMask |
                    ButtonReleaseMask,
                    GrabModeAsync, GrabModeAsync,
                    XtWindow(canvas),
                    XCreateFontCursor(XtDisplay(canvas),
                                      XC_crosshair));

        data.gc = xs_create_xor_gc(canvas);

	XtAppAddTimeOut(XtWidgetToApplicationContext(toplevel),2000,TimeOutCB,toplevel);

	gcvalues.foreground = BlackPixel(XtDisplay(canvas), 0);
	gcvalues.background = WhitePixel(XtDisplay(canvas), 0);
	SimpleGC = XCreateGC(XtDisplay(canvas), XtWindow(canvas),
				(GCForeground | GCBackground), &gcvalues);

	attrs.bit_gravity = ForgetGravity;
	valuemask = CWBitGravity;
	XChangeWindowAttributes( XtDisplay(canvas), XtWindow(canvas),
				valuemask, &attrs);

	XtAppMainLoop(app);

	exit(0);
}

GC xs_create_xor_gc(w)
    Widget        w;
{
XGCValues values;
GC    gc;
Arg   wargs[10];
XtSetArg(wargs[0], XtNforeground, &values.foreground);
XtSetArg(wargs[1], XtNbackground, &values.background);
XtGetValues(w, wargs,2);
values.foreground = values.foreground ^ values.background;
values.line_style = LineSolid; //LineOnOffDash
values.function = GXxor;
gc = XtGetGC(w,GCForeground | GCBackground |
             GCFunction | GCLineStyle, &values);
return gc;
}
//
//dec
//
void start_rubber_band(w,data,event)
Widget    w;
rubber_band_data  *data;
XEvent            *event;
{

if(magnet_lock == 1) return;

halt_update=1;
data->last_x = data->start_x = event->xbutton.x;
data->last_y = data->start_y = event->xbutton.y;
XDrawLine(XtDisplay(w), XtWindow(w), data->gc, data->start_x,
          data->start_y, data->last_x, data->last_y);
}
void track_rubber_band(w,data,event)
Widget    w;
rubber_band_data  *data;
XEvent            *event;
{
  
char x_command[100];
char y_command[100];
int dispx,dispy;

//float xval,yval;
	
if(magnet_lock == 1) return;

if(halt_update == 1)
  {
  rubberband=1;
  current_x = event->xbutton.x;
  current_y = event->xbutton.y;

  time=50;

  yval=6.0000;
  
    if(highres == 0) {
    //xval=current_x/100.0;
    //yval=(float)((900-current_y)/100.0)*1.15;
    txval=((float)current_x/(float)new_width)*XSCALE;
    tyval=((float)((new_height-20.0-current_y))/(float)(new_height-36.0))*YSCALE;
    //printf("\nx=%f y=%f",txval,tyval); 
    if(fabs(txval - xval) < .5) { 
    // &&  (fabs(tyval - yval) < 1.234)) {
         //xval=((float)current_x/(float)new_width)*XSCALE;
         //yval=((float)((new_height-20.0-current_y))/(float)(new_height-36.0))*YSCALE;
         xval=txval;
	 //yval=tyval;
	 }
    }
  if(highres == 1) {
    if(current_x > data->last_x)xval=xval+.004;
    if(current_x < data->last_x)xval=xval-.004;
    //if(current_y > data->last_y)yval=yval-.004;
    //if(current_y < data->last_y)yval=yval+.004;
    }
  if(highres == 2) {
    if(current_x > data->last_x)xval=xval+.001;
    if(current_x < data->last_x)xval=xval-.001;
    //if(current_y > data->last_y)yval=yval-.001;
    //if(current_y < data->last_y)yval=yval+.001;
    }

  if(highres == 3) {
    if(current_x > data->last_x)xval=xval+.00025;
    if(current_x < data->last_x)xval=xval-.00025;
    //if(current_y > data->last_y)yval=yval-.00025;
    //if(current_y < data->last_y)yval=yval+.00025;
    }
  if(highres == 4) {
    if(current_x > data->last_x) xval=xval+.00005;
    if(current_x < data->last_x) xval=xval-.00005;
    //yval=xval;
    }
   
  if(highres == 5) {
    if(current_x > data->last_x) {
       xval=xval+.004;
       //yval=yval+.004;
       }
    if(current_x < data->last_x) {
       xval=xval-.004;
       //yval=yval-.004;
       }
    }

  if(highres == 6) {
      dispx = current_x - data->last_x;
      dispy = current_y - data->last_y;
      if (abs(dispx) > abs(dispy)) {
         if (dispx > 0)xval=xval+.004;
	 else          xval=xval-.004; 
         }
      else if (abs(dispx) < abs(dispy)) {
         if (dispy > 0)yval=yval-.004;
         else          yval=yval+.004;
         }
    }
  
  if(yval < 0.01) yval = 0.01;
  if(xval < 0.01) xval = 0.01;
  
  
  ixval = (xval/xvalmax)*1000000;
  printf("ixval = %06d\n",ixval);
  sprintf(x_command,"da 0 %06d",ixval);

// write messages to the Danfysik                    
 
// write(fd, x_command, strlen(x_command));
write(fd, x_command, strlen(x_command));
write(fd, "\n", 1);
  //printf("\n x = %s \n",x_command);

	  
  data->last_x = event->xbutton.x;
  data->last_y = event->xbutton.y;
  }
}

void end_rubber_band(w,data,event)
Widget    w;
rubber_band_data  *data;
XEvent            *event;
{
int start,stop,len,j;
char x_command[100];
char y_command[100];

if(magnet_lock == 1) return;

if(rubberband  == 0) {
  //XDrawLine(XtDisplay(w), XtWindow(w), data->gc, data->start_x,
  //       data->start_y, data->last_x, data->last_y);
  //data->last_x = event->xbutton.x;
  //data->last_y = event->xbutton.y;
  //start=plot.start;
  //len=plot.stop-plot.start+1;
  if(event->xbutton.button == Button1){
    //plot.start = start + (len*((float)data->start_x/(float)new_width));
    //stop = start + (len*((float)data->last_x/(float)new_width));
    //plot.start=0;
    //plot.stop=1000;
    //printf("\n x = %d, y = %d\n",event->xbutton.x,event->xbutton.y);
    /*
    if(highres == 0) highres = 1;
    else if(highres == 1) highres = 2;
    else if(highres == 2) highres = 3;
    else if(highres == 4) highres = 1;
    else if(highres == 5) highres = 1;
    else if(highres == 6) highres = 1;
    */
    if(highres == 0) xval = xval-.016;
    else if(highres == 1) xval=xval-.004;
    else if(highres == 2) xval=xval-.001;
    else if(highres == 3) xval=xval-.00025;
    else if(highres == 4) xval=xval-.00005;
    else if(highres == 5) xval=xval-.004;
    else if(highres == 6) xval=xval-.004;
    }
  if(event->xbutton.button == Button3){
    //plot.start = 0;
    //plot.stop = 1000;
    /*
    if(highres == 4) highres = 0;
    else if(highres == 3) highres = 2;
    else if(highres == 2) highres = 1;
    else if(highres == 1) highres = 0;
    else if(highres == 5) highres = 0;
    else if(highres == 6) highres = 0;
    */
    if(highres == 0) xval = xval+.016;
    else if(highres == 1) xval=xval+.004;
    else if(highres == 2) xval=xval+.001;
    else if(highres == 3) xval=xval+.00025;
    else if(highres == 4) xval=xval+.00005;
    else if(highres == 5) xval=xval+.004;
    else if(highres == 6) xval=xval+.004;
    }
  //if(stop > (plot.start+1)) plot.stop=stop;
  if(event->xbutton.button == Button2){
    //plot.start=0;           
    //plot.stop=1000;         
    //highres = 0;
    if(highres == 0) highres = 1;
    else if(highres == 1) highres = 2;
    else if(highres == 2) highres = 3;
    else if(highres == 3) highres = 4;
    else if(highres == 4) highres = 0;
    }
  }
  ixval = (xval/xvalmax)*1000000;
  printf("ixval = %06d\n",ixval);
//sprintf(x_command,"da 0 %i",(xval*1000000)/xvalmax);
  sprintf(x_command,"da 0 %06d",ixval);
write(fd, x_command, strlen(x_command));
write(fd, "\n", 1);
rubberband=0;
halt_update=0;
}
 
void mouse_motion(w,data,event)
Widget    w;
rubber_band_data  *data;
XEvent            *event;
{
current_x = event->xbutton.x;
current_y = event->xbutton.y;
//printf("\nmouse x = %d y = %d \n",current_x,current_y);
mousemoved = 2;
}

void enter_window(w,data,event)
	Widget    w;
	//focus_change_data *data;
	XEnterWindowEvent   *event;
{
	//printf("\nevent focus change type=%d mode=%d detail=%d same_screen=%d focus=%d\n",event->type,event->mode,event ->detail,event->same_screen, event->focus);

}

void exit_window(w,data,event)
	        Widget    w;
		//focus_change_data *data;
		XLeaveWindowEvent   *event;
{

	//printf("\nevent focus change type=%d mode=%d detail=%d same_screen=%d focus=%d\n",event->type,event->mode,event ->detail,event->same_screen, event->focus);
	//printf("\nevent focus change type=%d \n",event->type);		
}

void forcedexit()
  {
//  if(shmdt(shmpltptr) == -1)
//    {
//    printf("\nshmdt failed in QuitCB.\n");
//   }
waittilldone:
//  if(shmctl(id,IPC_RMID,0) == -1)
//    {
//    printf("\nshmctl release failed in QUIT.\n");
//    goto waittilldone;
//   }
//printf("\n Shared memory id %d deallocated in daqxplot \n",id);
//XtCloseDisplay(XtDisplay(w));
  close(fd);
  exit(0);
  }



void QuitCB(w, client_data, call_data)
        Widget  w;
        struct dataplot *client_data;
        XmDrawingAreaCallbackStruct *call_data;
{
        XColor  col, unused;
        Colormap cmap = DefaultColormapOfScreen (XtScreen (w));

        extern GC       SimpleGC;
        char strbuff[100];
        Display *dsply;
        Window drawable;
        dsply = XtDisplay(w);
        drawable = XtWindow(w);

        static int flag;
        int x, y, b, k, z;
        int i;
        i = client_data;


        /* This is done to assure all the activate callbacks are complete
                before the program ends.  Refer to the X Window System
                Programming and Applications book by D.A. Young, pg 77. */

        switch(i) {

        case 1: /* Arm */

                flag = 0;       /* if we disarm now, nothing happens */
                break;

        case 2: /* Activate */

                flag = 1;       /* user released button, stand by to quit */
                break;

        case 3: /* Disarm */

                if (flag == 1) {        /* finished with things, now exit */

                        if(XtIsRealized(canvas))
                             XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
                        sprintf(strbuff,"Confirm Request to Quit. (y/n)");
                        XSetForeground(XtDisplay(w),SimpleGC,ouRed);
                        XDrawString(XtDisplay(canvas),XtWindow(canvas),SimpleGC,20,new_height/2,strbuff,strlen(strbuff));
                        //XDrawString(dsply,drawable,SimpleGC,20,100,strbuff,strlen(strbuff));
                        XSetForeground(dsply,SimpleGC,ouBlack);
                        z=rinptx (1, 0, 0, 1, &x, &y, &b, &k);
                        //printf("\n%d %d %d %d %d",x,y,b,k,z);
                        if(k!=29){
                            printf("\nExpecting 29 but found %i",k);
                            return 0;
                            }
                        XtCloseDisplay(XtDisplay(w));
                        close(fd);
                        exit(0);
                }

        }
}



void Ex1CB(w, client_data, call_data)
	Widget	w;
	int	*client_data;
	XmAnyCallbackStruct	*call_data;
{
	static int flag;
	int i;
        struct stat stbuf;
	i = client_data;

	/* This is done to assure all the activate callbacks are complete
		before the program ends.  Refer to the X Window System
		Programming and Applications book by D.A. Young, pg 77. */

	switch(i) {

	case 1:	/* Arm */
			
		flag = 0;	/* if we disarm now, nothing happens */
		break;

	case 2: /* Activate */

		flag = 1;	/* user released button, stand by to quit */
		break;

	case 3: /* Disarm */

                if (flag == 1)  /* finished with things, now exit */
                  {
                  if(stat("ex1.daq",&stbuf) != -1)
                    {
                    system("./rdaq_client localhost ex1\n");
                    }
                  else if(stat("Rbuttons",&stbuf) != -1)
                    {
                    system("./rdaq_client 10.0.0.11 ex1\n");
                    }
                  else  
                    {
                    plot.stop = plot.start + (((plot.stop-plot.start)+1)/4)-1;
                    if(XtIsRealized(canvas))
                      XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
		    }
                  }
	}
}
void Ex2CB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i, delta;
        struct stat stbuf;
        i = client_data;

        /* This is done to assure all the activate callbacks are complete
                before the program ends.  Refer to the X Window System
                Programming and Applications book by D.A. Young, pg 77. */

        switch(i) {

        case 1: /* Arm */

                flag = 0;       /* if we disarm now, nothing happens */
                break;

        case 2: /* Activate */

                flag = 1;       /* user released button, stand by to quit */
                break;

        case 3: /* Disarm */

                if (flag == 1)  /* finished with things, now exit */
                  {
                  if(stat("ex2.daq",&stbuf) != -1)
                    {
                    system("./rdaq_client localhost ex2\n");
                    }
                  else if(stat("Rbuttons",&stbuf) != -1)
                    {
                    system("./rdaq_client 10.0.0.11 ex2\n");
                    }
                  else
                    {
                    delta = (((plot.stop-plot.start)+1) / 4);
                    plot.start = plot.start + delta;
                    plot.stop = plot.start + delta - 1;
                    if(XtIsRealized(canvas))
                      XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
                    }
                  }

        }
}
void Ex3CB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i, delta;
        struct stat stbuf;
        i = client_data;

        /* This is done to assure all the activate callbacks are complete
                before the program ends.  Refer to the X Window System
                Programming and Applications book by D.A. Young, pg 77. */

        switch(i) {

        case 1: /* Arm */

                flag = 0;       /* if we disarm now, nothing happens */
                break;

        case 2: /* Activate */

                flag = 1;       /* user released button, stand by to quit */
                break;

        case 3: /* Disarm */

                if (flag == 1)  /* finished with things, now exit */
                  {
                  if(stat("ex3.daq",&stbuf) != -1)
                    {
                    system("./rdaq_client localhost ex3\n");
                    }
                  else if(stat("Rbuttons",&stbuf) != -1)
                    {
                    system("./rdaq_client 10.0.0.11 ex3\n");
                    }
                  else
                    {
                    delta = (((plot.stop-plot.start)+1) / 4);
                    plot.start = plot.start + (2 * delta);
                    plot.stop = plot.start + delta - 1;
                    if(XtIsRealized(canvas))
                      XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
                    }
                  }

        }
}
void Ex4CB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i, delta;
        struct stat stbuf;
        i = client_data;

        switch(i) {

        case 1: /* Arm */

                flag = 0;       /* if we disarm now, nothing happens */
                break;

        case 2: /* Activate */

                flag = 1;       /* user released button, stand by to quit */
                break;

        case 3: /* Disarm */

                if (flag == 1)  /* finished with things, now exit */
                  {
                  if(stat("ex4.daq",&stbuf) != -1)
                    {
                    system("./rdaq_client localhost ex4\n");
                    }
                  else if(stat("Rbuttons",&stbuf) != -1)
                    {
                    system("./rdaq_client 10.0.0.11 ex4\n");
                    }
                  else
                    {
                    delta = (((plot.stop-plot.start)+1) / 4);
                    plot.start = plot.start + (3 * delta);
                    plot.stop = plot.start + delta - 1;
                    if(XtIsRealized(canvas))
                      XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
                    }
                  }

        }
}

void ZoomCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
	int delta;
        i = client_data;

        if(magnet_lock == 1) return;

        switch(i) {

        case 1: /* Arm */
                flag = 0;       /* if we disarm now, nothing happens */
                break;
        case 2: /* Activate */
                flag = 1;       /* user released button, stand by to quit */
                break;
        case 3: /* Disarm */
                if (flag == 1) {        /* finished with things, now exit */
                   //if(rtype == 0) delta = (plot.stop - plot.start ) / 4;
		   //else delta = 64;
                   //plot.start = plot.start + delta;
                   //if((plot.stop - delta) > plot.start)plot.stop = plot.stop - delta;
                   //if(XtIsRealized(canvas))
	           if(highres == 0) highres=1;
		   else if(highres == 1) highres=2;
		   else if(highres == 2) highres=3;
		   else if(highres == 3) highres=4;
		   else if(highres == 5) highres=1;
		   else if(highres == 6) highres=1;
                   /*  
                   if(highres == 1) xval=xval+.004;
                   else if(highres == 2) xval=xval+.001;
                   else if(highres == 3) xval=xval+.00025;
                   else if(highres == 5) xval=xval+.004;
                   else if(highres == 6) xval=xval+.004;
                   */
                   //XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
                }

        }
}

void UnZoomCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
        int delta;
        i = client_data;

        if(magnet_lock == 1) return;
 
        switch(i) {

        case 1: /* Arm */
                flag = 0;       /* if we disarm now, nothing happens */
                break;
        case 2: /* Activate */
                flag = 1;       /* user released button, stand by to quit */
                break;
        case 3: /* Disarm */
                if (flag == 1) {        /* finished with things, now exit */
                   //if(rtype == 0) delta = (plot.stop - plot.start ) / 4;
                   //else delta = 64;
                   //plot.start = plot.start - delta;
                   //plot.stop = plot.stop + delta;
		   //if(plot.start < 0) plot.start = 0;
		   //if(plot.start < plot.istart) plot.start = plot.istart;
                   //if(plot.stop > plot.istop) plot.stop = plot.istop;
                   //if(XtIsRealized(canvas))
                   //XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
                   if(highres == 4) highres=3;
		   else if(highres == 3) highres=2;
		   else if(highres == 2) highres=1;
		   else if(highres == 1) highres=0;
		   else if(highres == 5) highres=0;
		   else if(highres == 6) highres=0;
                   /*
                   if(highres == 1) xval=xval+.004;
                   else if(highres == 2) xval=xval+.001;
                   else if(highres == 3) xval=xval+.00025;
                   else if(highres == 5) xval=xval+.004;
                   else if(highres == 6) xval=xval+.004;
                   */
                   //xval=xval+.00025;
                }

        }
}
  

void PanlCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
        int delta;
        i = client_data;

        switch(i) {

        case 1: /* Arm */
                flag = 0;       /* if we disarm now, nothing happens */
                break;
        case 2: /* Activate */
                flag = 1;       /* user released button, stand by to quit */
                break;
        case 3: /* Disarm */
                if (flag == 1) {        /* finished with things, now exit */
		   /*
                   if(rtype == 0) delta = (plot.stop - plot.start ) / 10;
                   else delta = 64;
                   plot.start = plot.start - delta;
		   //if (plot.start < 0) plot.start = 0;
                   if (plot.start < plot.istart) plot.start = plot.istart;
                   plot.stop = plot.stop - delta;
		   if (plot.stop < 0) plot.stop = 10;
                   if(XtIsRealized(canvas))
                   XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
		   */
		   save_xval = xval;
		   save_yval = yval;
                }

        }
}

void PanrCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
        int input;
        int delta;
        i = client_data;

        if(magnet_lock == 1) return;

        switch(i) {

        case 1: /* Arm */
                flag = 0;       /* if we disarm now, nothing happens */
                break;
        case 2: /* Activate */
                flag = 1;       /* user released button, stand by to quit */
                break;
        case 3: /* Disarm */
                if (flag == 1) {        /* finished with things, now exit */
		   /*
                   if(rtype == 0) delta = (plot.stop - plot.start ) / 10;
                   else delta = 64;
                   plot.start = plot.start + delta;
                   if (plot.start > plot.istop) plot.start = plot.istop - 10;
                   plot.stop = plot.stop + delta;
                   if (plot.stop < plot.start) plot.stop = plot.istop;
		   if (plot.stop > plot.istop) plot.stop = plot.istop;
                   if(XtIsRealized(canvas))
                   XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
		   */
		   xval = save_xval;
		   yval = save_yval;
                   if ((input = fopen("switcher.tandem.data","w")) != NULL) {
                      //fprintf(input,"%s %d %f %f\n","inflection",0,xval*3.0,yval*3.0);
                      fprintf(input,"%s %d %f %f\n","switcher",1,xval*SHUNT_FACTOR,yval);
                      fclose (input);
                      }
                }

        }
}

void AutoCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
        i = client_data;

        switch(i) {

        case 1: /* Arm */

                flag = 0;       /* if we disarm now, nothing happens */
                break;

        case 2: /* Activate */

                flag = 1;       /* user released button, stand by to quit */
                break;

        case 3: /* Disarm */

                if (flag == 1) {        /* finished with things, now exit */
		/*
                   if(plot.autoy == 0) plot.autoy = 1; 
                   else 
                      {
                      plot.autoy = 0;
                      plot.max = 8;
                      }   
                   if(XtIsRealized(canvas))
                   XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
                */
		highres = 4;
		}

        }
}

void YupCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
        i = client_data;

        switch(i) {

        case 1: /* Arm */

                flag = 0;       /* if we disarm now, nothing happens */
                break;

        case 2: /* Activate */

                flag = 1;       /* user released button, stand by to quit */
                break;

        case 3: /* Disarm */

                if (flag == 1) {        /* finished with things, now exit */
		   /*
                   plot.autoy = 0;
                   plot.max = plot.max * 2;
                   if(XtIsRealized(canvas))
                   XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
		   */
		   highres = 6;
                }

        }
}

void YdnCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
        i = client_data;

        switch(i) {

        case 1: /* Arm */

                flag = 0;       /* if we disarm now, nothing happens */
                break;

        case 2: /* Activate */

                flag = 1;       /* user released button, stand by to quit */
                break;

        case 3: /* Disarm */

                if (flag == 1) {        /* finished with things, now exit */
		   /*
                   plot.autoy = 0;
                   if(plot.max > 2)plot.max = plot.max / 2;
                   if(XtIsRealized(canvas))
                   XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
		   */
		   highres = 5;
                }

        }
}

void HomeCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
        i = client_data;

        switch(i) {

        case 1: /* Arm */

                flag = 0;       /* if we disarm now, nothing happens */
                break;

        case 2: /* Activate */

                flag = 1;       /* user released button, stand by to quit */
                break;

        case 3: /* Disarm */

                if (flag == 1) {        /* finished with things, now exit */

                   plot.autoy = 1;
                   //plot.start = 0;
                   plot.start = plot.istart;
                   plot.stop = plot.istop;
		   highres=0;
                   if(XtIsRealized(canvas))
		   XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
                }

        }
}
  
void RefCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
        int delta;
        i = client_data;

        switch(i) {

        case 1: /* Arm */
                flag = 0;       /* if we disarm now, nothing happens */
                break;
        case 2: /* Activate */
                flag = 1;       /* user released button, stand by to quit */
                break;
        case 3: /* Disarm */
                if (flag == 1) {        /* finished with things, now exit */
                   if(magnet_lock == 1) {
                      magnet_lock = 0;
                   }
                   else {
                      magnet_lock = 1;
                   }
                }

        }
}

void TimeOutCB(client_data, call_data )
	Widget client_data;
	XtIntervalId *call_data;
{
	//int time,j;
	int j,flag,input;
	float new_xval, new_yval;
	char device_name[100];
	char x_command[100];
	char y_command[100];
	
	new_xval = xval;
	new_yval = yval;
	
        if((refloops > 0) && (mousemoved == 0))
          {
          if(--refloops == 0)
            {
            //XtAppAddTimeOut(XtWidgetToApplicationContext(client_data),time,QuitCB,3);
            if(shmctl(id,IPC_RMID,0) == -1)
              {
              printf("\nshmctl release failed in QUIT.\n");
              }
            exit(0);
            }
          }
// 	if (mousemoved == 0) updatedata(&plot);
        //if(halt_update == 0)
        //  {
          if(XtIsRealized(canvas))
           XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
        //  }

     if(plot.autoy == 1) {
        plot.max = 16;
        for (j = plot.start; j <= plot.stop; j++) {
                while (plot.data[j] > plot.max) plot.max = plot.max << 1;
                }
        }

       /* 
	time = plot.stop - plot.start;
        if (time > 1024) time = 2000;
        if (time > 5000) time = 3000;
        if (time > 10000) time = 5000;
	if (time <  512) time = 512;
        if (rtype > 0) time = 2000;
        //time=500;

        time=50;
        
        rate_factor = (float)time/1000.0; 

        if (mousemoved > 0)
           {
           time=50;
           if(rtype > 0) time = 1000;
           mousemoved--;
           }
	*/
	if (rubberband == 1) time = 050;
	else { 
             time=50;
	     if ((input = fopen("switcher.tandem.data","r")) != NULL) {
	        fscanf(input,"%s %d %f %f",device_name,&flag,&new_xval,&new_yval);
		//printf("\n%s %d %f %f",device_name,flag,new_xval,new_yval);
		fclose (input);
	        }
	     if(flag == 1) {
		//xval = new_xval / 3.0;  
		//yval = new_yval / 3.0;
                xval = new_xval/SHUNT_FACTOR;
                ixval = (xval/xvalmax)*1000000;
                printf("ixval = %06d\n",ixval);
		//sprintf(x_command,"da 0 %i",(xval*1000000)/xvalmax);
                sprintf(x_command,"da 0 %06d",ixval);

	         // write messages to the Danfysik                    
		
		write(fd, x_command, strlen(x_command));
		write(fd, "\n", 1);

	        }
	     if ((input = fopen("switcher.tandem.data","w")) != NULL) {
	        //fprintf(input,"%s %d %f %f\n","inflection",0,xval*3.0,yval*3.0);
                fprintf(input,"%s %d %f %f\n","switcher",0,xval*SHUNT_FACTOR,yval);
	        fclose (input);
	        }
             }

	
	if(XtIsRealized(canvas))
              XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);
        XtAppAddTimeOut(XtWidgetToApplicationContext(client_data),time,TimeOutCB,client_data);
} 

void RedrawCB( w, client_data, call_data )
	Widget	w;
	struct dataplot	*client_data;
	XmDrawingAreaCallbackStruct *call_data;
{
	XColor  col, unused;
        Colormap cmap = DefaultColormapOfScreen (XtScreen (w));

	extern GC	SimpleGC;
	int x, y, oldx, oldy, i, j, sum, width, z, dz;
        int slx, sly, sli, slj, slwidth, sly0, slx0, temp;
        int rx,ry,rb,rk;
	float xstep, ystep;
        float slxstep, slystep;
        int curmouse_x,curmouse_y,ix,iy,iz,thresh;
	char strbuff[100];
	Display *dsply;
	Window drawable;
        int size_thresh = 250; // change labeling if less than this wide
        int rate=0;
	dsply = XtDisplay(w);
	drawable = XtWindow(w);
	  
        if(ouColors == 0)
          {
	  for (j=0; j<16; j++)
                {
                XAllocNamedColor (dsply,cmap,colors[j],&col,&unused);

                ouZcolor[j]=col.pixel;
                }

	  XAllocNamedColor (dsply,cmap,"White",&col,&unused);
          ouWhite=col.pixel;
          XAllocNamedColor (dsply,cmap,"Black",&col,&unused);
          ouBlack=col.pixel;
          XAllocNamedColor (dsply,cmap,"Gray",&col,&unused);
          ouGray=col.pixel;
          XAllocNamedColor (dsply,cmap,"Magenta",&col,&unused);
          ouMagenta=col.pixel;
          XAllocNamedColor (dsply,cmap,"Cyan",&col,&unused);
          ouCyan=col.pixel;
          XAllocNamedColor (dsply,cmap,"Yellow",&col,&unused);
          ouYellow=col.pixel;
          XAllocNamedColor (dsply,cmap,"Red",&col,&unused);
          ouRed=col.pixel;
          XAllocNamedColor (dsply,cmap,"Blue",&col,&unused);
          ouBlue=col.pixel;
	  //XAllocNamedColor (dsply,cmap,"Green",&col,&unused);
          XAllocNamedColor (dsply,cmap,"DarkSlateGray4",&col,&unused);
	  ouGreen=col.pixel;

  	  ouColors = 1;
	  
	  }
	
	i = plot.max;
/*	xs_wprintf(plot.info, "%d", i); */ 

	/* Get the window size */
	XtVaGetValues(w,
		XmNwidth, &new_width,
		XmNheight, &new_height,
		NULL);


     if(plot.autoy == 1) {
	plot.max = 16;
        for (j = plot.start; j <= plot.stop; j++) {
                while (plot.data[j] > plot.max) plot.max = plot.max << 1;
                }
        }                  

	xstep = new_width;
	xstep = xstep / (plot.stop - plot.start + 1);
	ystep = new_height - 20 - 15; // -15 added to allow for lettering at top of plot
	ystep = ystep / plot.max; 
        if(plot.max <= 8) ystep = ystep / 100.0;  // for log plot

/*	printf("Redraw: xstep = %f ystep = %f\n",xstep,ystep); */
/*	printf("start = %d   stop = %d   max = %d\n", plot.start, plot.stop, plot.max); */
	sum = 0;
	oldx = 0;   
	oldy = new_height - 20;
        dz = 2;
        //if((plot.stop-plot.start) > 500) dz=1;
        //if((plot.stop-plot.start) < 100) dz=3;
        i = 0; 
	if(rtype == 0)
            {
            //XAllocNamedColor (dsply,cmap,"Black",&col,&unused);
            //XSetForeground(dsply,SimpleGC,col.pixel);
            for (j = plot.start; j <= plot.stop; j++) 
		{
                sum += plot.data[j];
                // if(j == mouse_channel)mouse_value=plot.data[j];
                if(plot.max > 8)
                   //quadmod dataline
                   y = (new_height - 20)/2;
                   //y = (new_height - 20) - (plot.data[j] * ystep);
                else
                   y = (new_height - 20) -
                       ((log10((double)plot.data[j]+1) * ystep) * 100.0);
                x = i * xstep;
                if(y > 15)
                  {
                  if(j != mouse_channel)
                    {
                    //quadmod dataline
                    //XDrawLine(dsply,drawable,SimpleGC,x,y-dz,x,y+dz);
		    //XDrawLine(dsply,drawable,SimpleGC,x-dz,y,x+dz,y);
                    //XDrawLine(dsply,drawable,SimpleGC,x,y-dz,x,y-dz);
                    //XDrawLine(dsply,drawable,SimpleGC,x,y+dz,x,y+dz);
                    }
                  else 
                    {
                    //XAllocNamedColor (dsply,cmap,"Magenta",&col,&unused);
			    
                    if(highres == 0) {
			    XSetForeground(dsply,SimpleGC,ouRed);
			    sprintf(strbuff,"Grab %6.4f Amps",SHUNT_FACTOR*xval);
		    }
		    else if(highres == 1) {
			    XSetForeground(dsply,SimpleGC,ouGreen);
			    sprintf(strbuff,"Med %6.4f Amps",SHUNT_FACTOR*xval);
		    }
		    else if(highres == 2) {
			    XSetForeground(dsply,SimpleGC,ouBlue);
			    sprintf(strbuff,"Fine %6.4f Amps",SHUNT_FACTOR*xval);
		    }
		    else if(highres == 3) {
		            XSetForeground(dsply,SimpleGC,ouMagenta);
		            sprintf(strbuff,"ExtraFine %6.4f Amps",SHUNT_FACTOR*xval);
		            }
                    else if(highres == 4) {
                            XSetForeground(dsply,SimpleGC,ouBlack);
                            sprintf(strbuff,"SuperFine %6.4f Amps",SHUNT_FACTOR*xval);
                    }
		    else if(highres == 5) {
                            XSetForeground(dsply,SimpleGC,ouCyan);
                            sprintf(strbuff,"dA = dB %6.4f Amps",SHUNT_FACTOR*xval);
                            } 
                    else if(highres == 6) {
                            XSetForeground(dsply,SimpleGC,ouGray);
                            sprintf(strbuff,"A OR B %6.4f Amps",SHUNT_FACTOR*xval);
                            }

                    XDrawLine(dsply,drawable,SimpleGC,mouse_channel,new_height-20-mouse_value-15,mouse_channel,new_height-20-mouse_value+15);
                    XDrawLine(dsply,drawable,SimpleGC,mouse_channel-15,new_height-20-mouse_value,mouse_channel+15,new_height-20-mouse_value); 
		    XDrawArc(dsply,drawable,SimpleGC,mouse_channel-8,new_height-20-mouse_value-8, 16,  16, 0, 360*64);
                    XDrawLine(dsply,drawable,SimpleGC,mouse_channel-7,new_height-15-mouse_value,mouse_channel,new_height-5-mouse_value);
                    XDrawLine(dsply,drawable,SimpleGC,mouse_channel+7,new_height-15-mouse_value,mouse_channel,new_height-5-mouse_value);
		    //XDrawArc(dsply,drawable,SimpleGC,mouse_channel-25,new_height-20-mouse_value-25, 50, 50, 0, 360*64);
		    // indicate expanded mode
		    //XAllocNamedColor (dsply,cmap,colors[j],&col,&unused);
		    //XSetForeground(dsply,SimpleGC,ouZcolor[highres+3]);
		    //XFillRectangle(dsply,drawable,SimpleGC,x,y,new_width/20,new_height/20);
		    //sprintf(strbuff,"%d",highres);
		    //XSetForeground(dsply,SimpleGC,ouBlack);
		    //XDrawString(dsply,drawable,SimpleGC,x+1,y+(new_height/20)-2,strbuff,strlen(strbuff));
                    //XAllocNamedColor (dsply,cmap,"Black",&col,&unused);
                    XSetForeground(dsply,SimpleGC,ouBlack);
		    XDrawString(dsply,drawable,SimpleGC,mouse_channel,new_height-60-mouse_value+22,strbuff,strlen(strbuff));
                    }

                  if(plot.data[524287] == 524287) // overlay defined
                    {
                    if(plot.max > 8)
                       y = (new_height - 20) - (plot.data[j+524288] * ystep);
                    else
                       y = (new_height - 20) -
                           ((log10((double)plot.data[j+524288]+1) * ystep) * 100.0);
                    XSetForeground(dsply,SimpleGC,ouBlue);
                    XDrawLine(dsply,drawable,SimpleGC,x,y-dz,x,y+dz);
                    XDrawLine(dsply,drawable,SimpleGC,x-dz,y,x+dz,y);
                    XSetForeground(dsply,SimpleGC,ouBlack);
                    }
                  } 
                i++;
                oldx = x;
                oldy = y;
                }
            }
        if((rtype == 1) || (rtype == 3))
            {
            sum = 0;
            i = 0;
            slwidth=new_width;
            new_width=(float)new_width-((float)new_width*0.2);
            x=(float)new_width*0.038;
            y=(float)new_height*0.03;
            xstep=(float)x*1.7;
            for (j=1; j<16; j++)
                {
                //XAllocNamedColor (dsply,cmap,colors[j],&col,&unused);
                XSetForeground(dsply,SimpleGC,ouZcolor[j]);
                XFillRectangle(dsply,drawable,SimpleGC,x,y,new_width/20,new_height/20);
                sprintf(strbuff,"%d",plot.max*j/16);
                //XAllocNamedColor (dsply,cmap,"Black",&col,&unused);
                XSetForeground(dsply,SimpleGC,ouBlack);
                XDrawString(dsply,drawable,SimpleGC,x+1,y+(new_height/20)-2,strbuff,strlen(strbuff));
                x += xstep;
                }
            xstep = ((float)new_width*0.9)/64.0;
            ystep = ((float)new_height*0.8)/64.0;
            curmouse_x=((((float)current_x-(float)new_width*0.05)/
                       ((float)new_width*0.9))*64.0);
            curmouse_y=64.0-(((float)current_y-(ystep/2.0)-((float)new_height*0.1))/ystep);
            //XAllocNamedColor (dsply,cmap,"White",&col,&unused);
            XSetForeground(dsply,SimpleGC,ouWhite);
            XDrawRectangle(dsply,drawable,SimpleGC,
                          (float)new_width*0.05,(float)new_height*0.1+ystep,
                          64*xstep,64*ystep);
            //XAllocNamedColor (dsply,cmap,"Black",&col,&unused);
            for (j = plot.start; j <= plot.stop; j++) 
                { 
                sum += plot.data[j];
                iy = j/64;
                ix = j-(64*iy);
                //x = ((j-(64*(j/64))) * xstep)+(float)new_width*0.05;
                x = (ix * xstep)+(float)new_width*0.05;
                y = (((float)new_height*0.9) - ((j/64) * ystep));
                //if ((ix==curmouse_x) && (iy==curmouse_y))
                //      mouse_value=plot.data[j];
                z=((float)plot.data[j] / (float)plot.max)*XSCALE;
                if(z > 15) z = 15;
                //if(z > 0)
                if(plot.data[j] > 0)
                    {
                    if(z > 0) {
                        //dec XAllocNamedColor (dsply,cmap,colors[z],&col,&unused);
                        //dec XSetForeground(dsply,SimpleGC,col.pixel);
                        XSetForeground(dsply,SimpleGC,ouZcolor[z]);
                        XFillRectangle(dsply,drawable,SimpleGC,x,y,xstep*0.8,ystep*0.8);
                    }
                    /*
                    else{
                        //XAllocNamedColor (dsply,cmap,"Gray",&col,&unused);
                        XSetForeground(dsply,SimpleGC,ouGray);
                        XFillRectangle(dsply,drawable,SimpleGC,x,y,xstep*0.2,ystep*0.2);
                        }
                    */
                    if ((ix==curmouse_x) && (iy==curmouse_y))
                       {
                       //XAllocNamedColor (dsply,cmap,"White",&col,&unused);
                       XSetForeground(dsply,SimpleGC,ouWhite);
                       XDrawRectangle(dsply,drawable,SimpleGC,
                            x-(0.1*xstep), y-(0.1*ystep),xstep,ystep);
                       slxstep = (float)new_width*0.24;
                       slxstep = slxstep / 64;
                       slystep = (new_height - 20)/1.5;
                       slystep = slystep / (2.0*plot.max);
                       sli=0;
                       //XAllocNamedColor (dsply,cmap,"Cyan",&col,&unused);
                       XSetForeground(dsply,SimpleGC,ouCyan);
                       sprintf(strbuff,"x(y=%d) z(%d,%d)=%d",iy,ix,iy,mouse_value);
                       sly0=new_height-(new_height*0.1);
                       XDrawString(dsply,drawable,SimpleGC,
                            new_width,new_height-(new_height*.06), strbuff,strlen(strbuff));
                       XDrawLine(dsply,drawable,SimpleGC,
                                new_width,sly0,
                                new_width+64*slxstep,sly0);
                       XDrawLine(dsply,drawable,SimpleGC,
                                new_width,sly0,new_width,new_height/1.9);
                       for (slj=iy*64; slj<(iy*64)+64; slj++)
                            {
                            sly = (new_height-(new_height*0.1))-(plot.data[slj] * slystep);
                            slx = (float)new_width + (sli * slxstep);
                            if(sly <= (new_height/1.9)) sly = (new_height/1.9);
                            XDrawLine(dsply,drawable,SimpleGC,slx,sly-1,slx,sly+1);
                            XDrawLine(dsply,drawable,SimpleGC,slx-1,sly,slx+1,sly);
                            if(sli==ix) 
                                { 
                                //XAllocNamedColor (dsply,cmap,"Yellow",&col,&unused);
                                XSetForeground(dsply,SimpleGC,ouYellow);
                                XDrawLine(dsply,drawable,SimpleGC,slx,sly-5,slx,sly+5);
                                XDrawLine(dsply,drawable,SimpleGC,slx-5,sly,slx+5,sly);
                                //XAllocNamedColor (dsply,cmap,"Cyan",&col,&unused);
                                XSetForeground(dsply,SimpleGC,ouCyan);
				XDrawLine(dsply,drawable,SimpleGC,
                                         (float)new_width*0.05,y+ystep/2,
                                         new_width-new_width*0.05,y+ystep/2);
                                sprintf(strbuff,"Y=%d",iy);
                                XDrawString(dsply,drawable,SimpleGC,
                                     0,y+ystep/2,
                                     strbuff,strlen(strbuff));
                                }
                            sli++;
                            }
                       sli=0;
                       //XAllocNamedColor (dsply,cmap,"Yellow",&col,&unused);
                       XSetForeground(dsply,SimpleGC,ouYellow);
                       sprintf(strbuff,"y(x=%d) z(%d,%d)=%d",ix,ix,iy,mouse_value);
                       sly0=(new_height-(new_height*0.1))/2;
                       XDrawString(dsply,drawable,SimpleGC,
                            new_width,(new_height/2), strbuff,strlen(strbuff));
                       XDrawLine(dsply,drawable,SimpleGC,
                                new_width,sly0,
                                new_width+64*slxstep,sly0);
                       XDrawLine(dsply,drawable,SimpleGC,
                                new_width,sly0,new_width,0.2*sly0);
                       //XAllocNamedColor (dsply,cmap,"White",&col,&unused);
                                XSetForeground(dsply,SimpleGC,ouWhite);
                       sprintf(strbuff,"Z Max = %d",plot.max);
                       XDrawString(dsply,drawable,SimpleGC,
                            new_width,0.1*sly0,strbuff,strlen(strbuff));
                       //XAllocNamedColor (dsply,cmap,"Yellow",&col,&unused);
                                XSetForeground(dsply,SimpleGC,ouYellow);
                       for (slj=ix; slj<4096; slj+=64)
                            {
                            sly = sly0-(plot.data[slj] * slystep);
                            slx = (float)new_width + (sli * slxstep);
                            if(sly <= 0.2*sly0) sly = 0.2*sly0;
                            XDrawLine(dsply,drawable,SimpleGC,slx,sly-1,slx,sly+1);
                            XDrawLine(dsply,drawable,SimpleGC,slx-1,sly,slx+1,sly);
                            if(sli==iy) 
                                {
                                //XAllocNamedColor (dsply,cmap,"White",&col,&unused);
                                XSetForeground(dsply,SimpleGC,ouWhite);
                                XDrawLine(dsply,drawable,SimpleGC,slx,sly-5,slx,sly+5);
                                XDrawLine(dsply,drawable,SimpleGC,slx-5,sly,slx+5,sly);
                                //XAllocNamedColor (dsply,cmap,"Yellow",&col,&unused);
                                XSetForeground(dsply,SimpleGC,ouYellow);
                                XDrawLine(dsply,drawable,SimpleGC,
                                          x+xstep/2,(((float)new_height*0.9)-(63*ystep)),
                                          x+xstep/2,(((float)new_height*0.9)+ystep));
                                sprintf(strbuff,"X=%d",ix);
                                XDrawString(dsply,drawable,SimpleGC,
                                     x+xstep/2,(((float)new_height*0.9)+25),
                                     strbuff,strlen(strbuff));
                                }
                            sli++;
                            }
                       }
                    //XAllocNamedColor (dsply,cmap,"Black",&col,&unused);
                    //XSetForeground(dsply,SimpleGC,col.pixel);
                    }
                i++;
		oldx = x;
		oldy = y;
		}
            //XAllocNamedColor (dsply,cmap,"White",&col,&unused);
            XSetForeground(dsply,SimpleGC,ouWhite);
            new_width=slwidth;
            }
        if((rtype == 2) || (rtype == 3))
            {
            sum = 0;
            oldx = 0;  
            oldy = new_height - 20;
            ystep = new_height - 20;
            ystep = ystep / plot.max;
            i = 0;
            //xstep=xstep/2.0;
            thresh=((float)plot.max*0.01)+1;
            ystep=ystep/2.0;
            //XAllocNamedColor (dsply,cmap,"White",&col,&unused);
            XSetForeground(dsply,SimpleGC,ouWhite);
            xstep = ((float)new_width*0.9)/128.0;
            //ystep = ((float)new_height*0.8)/64.0;
            for (j = plot.start; j <= plot.stop; j++)
                {
                iy = j/64;
                ix = j-(64*iy);
                iz = plot.data[j];
                //x = ((j-(64*(j/64))) * xstep)+(float)new_width*0.05;
                x = ((ix * xstep)+(float)new_width*0.05)+((float)iy*xstep);
                //y = ((((float)new_height*0.9) - ((j/64) * ystep)))-((float)iy*ystep);
                
                
                sum += iz;
                //if(j == mouse_channel)mouse_value=plot.data[j];
                y = (new_height - 40) - (iz * ystep)-
                    (((float)(new_height-40)/128.0)*iy);
                if(iz == 0) XDrawPoint(dsply,drawable,SimpleGC,x,y);
                else if(iz < thresh) XDrawLine(dsply,drawable,SimpleGC,x,y,x+1,y);
                else 
                  {
                  XDrawLine(dsply,drawable,SimpleGC,x,y-1,x,y+1);
                  XDrawLine(dsply,drawable,SimpleGC,x-1,y,x+1,y);
                  if(ix != 0)XDrawLine(dsply,drawable,SimpleGC,oldx,oldy,x,y);
                  }
                //i++;
                oldx = x;
                oldy = y;
                }
            }
  //mouse_channel = (((float)current_x/(float)new_width)*((plot.stop-plot.start)+1))+plot.start;
  //mouse_channel=xval*100.0;
  //mouse_value=yval*100.0;
  //dec change scale factor
  //mouse_channel=(xval/XSCALE)*(float)new_width;
  mouse_channel=(xval/XSCALE)*(float)new_width;
  mouse_value=(yval/YSCALE)*(float)(new_height-36);
  //mouse_value=0.5*(float)(new_height-36);
  if(rtype == 0)
    {
    if(new_width > size_thresh)
      {
      //sprintf(strbuff,"Start = %d",plot.start);
      sprintf(strbuff,"0");
      XDrawString(dsply,drawable,SimpleGC,2,new_height-5,strbuff,strlen(strbuff));
      //sprintf(strbuff,"Cursor(chan %d = %d)",mouse_channel,mouse_value);
      sprintf(strbuff,"50");
      XDrawString(dsply,drawable,SimpleGC,1*((new_width-(strlen(strbuff)*7)))/5,new_height-5,strbuff,strlen(strbuff));
      sprintf(strbuff,"100");
      XDrawString(dsply,drawable,SimpleGC,2*((new_width-(strlen(strbuff)*7)))/5,new_height-5,strbuff,strlen(strbuff));
      sprintf(strbuff,"150");
      XDrawString(dsply,drawable,SimpleGC,3*((new_width-(strlen(strbuff)*7)))/5,new_height-5,strbuff,strlen(strbuff));
      sprintf(strbuff,"200");
      XDrawString(dsply,drawable,SimpleGC,4*((new_width-(strlen(strbuff)*7)))/5,new_height-5,strbuff,strlen(strbuff));
      sprintf(strbuff,"250");
      XDrawString(dsply,drawable,SimpleGC,(new_width-(strlen(strbuff)*7)),new_height-5,strbuff,strlen(strbuff));
      sprintf(strbuff,"Amps");
      if(magnet_lock == 1) sprintf(strbuff,"Locked");
      if(magnet_lock == 1) XSetForeground(dsply,SimpleGC,ouRed);
      XDrawString(dsply,drawable,SimpleGC,(new_width-5-(strlen(strbuff)*7))/2,new_height-5,strbuff,strlen(strbuff));
      XDrawLine(dsply,drawable,SimpleGC,0,15,new_width,15);
      }
    else
      {
      sprintf(strbuff,"%d",plot.start);
      XDrawString(dsply,drawable,SimpleGC,10,new_height-5,strbuff,strlen(strbuff));
      //sprintf(strbuff,"(Chan%f=%f)",xval,yval);
      XDrawString(dsply,drawable,SimpleGC,(new_width-(strlen(strbuff)*7))/2,new_height-5,strbuff,strlen(strbuff));
      sprintf(strbuff,"%d",plot.stop);
      XDrawString(dsply,drawable,SimpleGC,new_width-5-(strlen(strbuff)*7),new_height-5,strbuff,strlen(strbuff));
      XDrawLine(dsply,drawable,SimpleGC,0,15,new_width,15);
      }
    }
  else
    {
    sprintf(strbuff,
    "Ystart = %d * MaxCounts = %d * Cursor(x,y,counts) = (%d,%d,%d) * SumCounts = %d * YStop = %d",
    plot.start/64,plot.max,curmouse_x,curmouse_y,mouse_value,sum,plot.stop/64);
    XDrawString(dsply,drawable,SimpleGC,10,new_height-5,strbuff,strlen(strbuff));
    }

  rate = (sum-previous_sum);
  rate = (float)rate/rate_factor;

  if((rtype == 0) && (plot.max > 8))
    {
    //if(new_width > size_thresh) sprintf(strbuff,"MaxCounts = %d",plot.max);
    //sprintf(strbuff,"6 Amps");
    //else sprintf(strbuff,"Y=%d",plot.max);
    //sprintf(strbuff,"Analyzer Magnet Current = %7.4f Amps (%8.6f MHz)",SHUNT_FACTOR*xval,0.259875*SHUNT_FACTOR*xval);
    sprintf(strbuff,"Switcher Magnet Current = %7.4f Amps",SHUNT_FACTOR*xval);
    XDrawString(dsply,drawable,SimpleGC,10,12,strbuff,strlen(strbuff));
    //if(new_width > size_thresh) 
    //  sprintf(strbuff,"Sum = %d Counts, %d Channels",sum,plot.stop-plot.start+1);
    //else sprintf(strbuff,"Sum=%d",sum);
    //XDrawString(dsply,drawable,SimpleGC,new_width-5-(strlen(strbuff)*7),
    //           12,strbuff,strlen(strbuff));
    //if(new_width > size_thresh)
    //  {
      //sprintf(strbuff,"Rate=%d",rate);
      //XDrawString(dsply,drawable,SimpleGC,new_width/3,
      //         12,strbuff,strlen(strbuff));
    //  }
    //else
    //  {
    //  sprintf(strbuff,"Rate=%d",rate);
    //  XDrawString(dsply,drawable,SimpleGC,(new_width-(strlen(strbuff)*7))/2,
    //           12,strbuff,strlen(strbuff));
    //  }
    // section to draw grid for linear single parameter histogram 
    /*
    XSetForeground(dsply,SimpleGC,ouGray);
    for (i=1;i<4;i++) XDrawLine(dsply,drawable,SimpleGC,0,15+(i*((new_height-20-15)/4)),
                      10,15+(i*((new_height-20-15)/4)));
    for (i=1;i<4;i++) XDrawLine(dsply,drawable,SimpleGC,new_width-10,15+(i*((new_height-20-15)/4)),
                      new_width,15+(i*((new_height-20-15)/4)));
    */
    XSetForeground(dsply,SimpleGC,ouBlack);
    }
  else if((rtype == 0) && (plot.max <= 8))
    {
    if(new_width > size_thresh) sprintf(strbuff,"MaxCounts = %d Cycle Log",plot.max);
    else sprintf(strbuff,"Log %d",plot.max);
    XDrawString(dsply,drawable,SimpleGC,10,12,strbuff,strlen(strbuff));
    if(new_width > size_thresh) 
      sprintf(strbuff,"Sum = %d Counts in %d Channels",sum,plot.stop-plot.start+1);
    else sprintf(strbuff,"Sum=%d",sum);
    XDrawString(dsply,drawable,SimpleGC,new_width-5-(strlen(strbuff)*7),
               12,strbuff,strlen(strbuff));
    if(new_width > size_thresh)
      {
      sprintf(strbuff,"Rate=%d",rate);
      XDrawString(dsply,drawable,SimpleGC,new_width/3,
               12,strbuff,strlen(strbuff));
      }
    else
      {
      sprintf(strbuff,"Rate=%d",rate);
      XDrawString(dsply,drawable,SimpleGC,(new_width-(strlen(strbuff)*7))/2,
               12,strbuff,strlen(strbuff));
      }
    XSetForeground(dsply,SimpleGC,ouGray);
    for (i=1;i<plot.max;i++) XDrawLine(dsply,drawable,SimpleGC,0,15+(i*((new_height-20-15)/plot.max)),
                      new_width,15+(i*((new_height-20-15)/plot.max)));
    XSetForeground(dsply,SimpleGC,ouBlack);
    }

  XDrawLine(dsply,drawable,SimpleGC,0,new_height-20,new_width,new_height-20);
  XDrawLine(dsply,drawable,SimpleGC,new_width/5,new_height-18,new_width/5,
      new_height-22);
  XDrawLine(dsply,drawable,SimpleGC,(2*new_width)/5,new_height-18,(2*new_width)/5,
      new_height-22);
  XDrawLine(dsply,drawable,SimpleGC,(3*new_width)/5,new_height-18,(3*new_width)/5,
      new_height-22);
  XDrawLine(dsply,drawable,SimpleGC,(4*new_width)/5,new_height-18,(4*new_width)/5,
      new_height-22);
  XDrawLine(dsply,drawable,SimpleGC,(5*new_width)/5,new_height-18,(5*new_width)/5,
      new_height-22);
  //XDrawLine(dsply,drawable,SimpleGC,new_width/2,new_height-18,new_width/2,
  //    new_height-22);
  //XAllocNamedColor (dsply,cmap,"Black",&col,&unused);

  XSetForeground(dsply,SimpleGC,ouCyan);

  if(rate > 1000) XSetForeground(dsply,SimpleGC,ouBlue);
  if(rate > 2000) XSetForeground(dsply,SimpleGC,ouYellow);
  if(rate > 3000) XSetForeground(dsply,SimpleGC,ouMagenta);
  if(rate > 4000) XSetForeground(dsply,SimpleGC,ouRed);
 
  rate = ((float)(rate)/5000.0)*new_width;
  if(rate > new_width) rate = new_width;

  XDrawLine(dsply,drawable,SimpleGC,0,16,rate,16);
  XDrawLine(dsply,drawable,SimpleGC,0,17,rate,17);
  XSetForeground(dsply,SimpleGC,ouBlack);
  previous_sum = sum;

}

void set_color(w, client_data, call_data)
   	Widget w;
        XtPointer client_data;
        XtPointer call_data;
{
   String color = (String) client_data;
   Display *dpy = XtDisplay (w);
   Colormap cmap = DefaultColormapOfScreen (XtScreen (w));
   XColor  col, unused;
   /*
   if(!XAllocNamedColor (dpy,cmap,color,&col,&unused)) {
      char buf[32];
      sprintf (buf, "Can't alloc %s", color);
      XtWarning (buf);
      return;
   }
   */
   XSetForeground (dpy,SimpleGC,col.pixel);
}

void ResizeCB( w, client_data, call_data )
	Widget	w;
	int	*client_data;
	XmDrawingAreaCallbackStruct *call_data;
{

/*	printf("ResizeCB() called %d, client_data = %d, reason=%d\n",
			CBcount, client_data,call_data->reason);
*/
	CBcount++;

}

int readdata(disk,tag,plot)
	int	disk,tag;
	struct dataplot *plot;
{
	char 	fname[128];
	int	rdisk,rtag,rstart,rstop,rdate,rtime;
	int	i,j,datum;
/*
	printf("Reading data for disk %d tag %d", disk,tag);
  
	sprintf(fname,"data/plotbuffer");

	if ((input = fopen(fname,"r")) == NULL) {
		perror("data file open failed");
		return(-1);
	}

	fscanf(input,"%d %d %d %d %d %d",&rdisk,&rtag,&rstart,&rstop,
					&rdate,&rtime);
  
	printf("readdata: disk %d tag %d start %d stop %d date %d time %d\n",
			rdisk, rtag, rstart, rstop, rdate, rtime);
*/
	// printf("readdata shmatt id = %d. \n",id);
	/*
	if((int) (shmpltptr = (char *) shmat(id,0,0)) == -1)
		{
		printf("\nshmatt failed in readdata.\n");
		exit(0);
		}
	*/
#ifdef DEBUGSHM
        printf("\nREADdata attaching %d\n",id);
#endif
	//plotbufferptr = (struct plotbuffer *) shmpltptr;
	//rdate = plotbufferptr->savdat;
	//rtime = plotbufferptr->savtim;
	//rstart = plotbufferptr->start;
	//rstop = plotbufferptr->stop;
        //rtype = plotbufferptr->type;
        //if(rtype == 5) rtype = 0;
	rstart=0;
	rstop=1000;
	rtype=0;
	if(rtype != 1) rtype = 0;
        i = (int)rstart;
//	i = 0;
/* 	fscanf(input,"%8d",&datum);
	plot->min = datum;
*/
	//datum=plotbufferptr->data[i];
	plot->min = 0;
	plot->max = 1000;
	plot->data[i++] = 0;
	for ( j = rstart+1; j <= rstop; j++) {
		//datum = plotbufferptr->data[i];    
		plot->data[i++] = j;
/*		if (datum < plot->min) plot->min = datum;
*/
		//if (datum > plot->max) plot->max = datum;
	}
	plot->start = rstart;
        plot->istart = rstart;
	plot->stop = rstop;
        plot->istop = rstop;
/*
	plot->start = 0;
	plot->istart = 0;
	plot->stop = i - 1;
	plot->istop = i - 1;
*/
	plot->autoy = 1;
/*	printf("Maximum counts %d\n",plot->max); */
/*      shmpltptr->inuse = 0; */
	//plotbufferptr->inuse = 0;
//	if(shmdt(shmpltptr) == -1)
//	{
//	printf("\nshmdt failed in readdata.\n");
//	}	
#ifdef DEBUGSHM
        printf("\nREADdata detatching %d\n",id);
#endif
/*	if(shmctl(id,IPC_RMID,0) == -1)
	{
	printf("\nshmctl release failed in readdata.\n");
	}
*/
	return(0);
}


int updatedata(plot)
        struct dataplot *plot;
{
        int     rdisk,rtag,rstart,rstop,rdate,rtime;
        int     i,j,datum;
        if(realtime == 0) return(0);
  	//printf("signalling process %d to refresh\n",parent); 
	//kill(parent,SIGUSR2);
        if((int) (shmpltptr = (char *) shmat(id,0,0)) == -1)
                {
                printf("\nshmatt failed in updatedata. id = %d.\n",id);
                exit(0);
                }
#ifdef DEBUGSHM
        printf("\nUPDATEdata attaching %d\n",id);
#endif
        plotbufferptr = (struct plotbuffer *) shmpltptr;
        plotbufferptr->refid = 0;
        plotbufferptr->inuse = 1;
        kill(parent,SIGUSR2);
        for(i=0;i<100000000;i++)
          {
          if(plotbufferptr->inuse == 0) goto active;
          }
        realtime = 0;
        goto notactive;
active:
#ifdef DEBUGSHM
        printf("\nFound Active. id = %d, returned refid = %d\n",id,plotbufferptr->refid);
#endif
        if(plotbufferptr->refid == id)
          {
          rstart = plotbufferptr->start;
          rstop = plotbufferptr->stop;
          i = 0;
          datum=plotbufferptr->data[i];
          plot->min = 0;
          /* plot->max = datum; */
          plot->data[i++] = datum;
          for ( j = rstart+1; j <= rstop; j++) 
            {
            datum = plotbufferptr->data[i];
            plot->data[i++] = datum;
            /* if (datum > plot->max) plot->max = datum; */
            }
          //printf("\nrefresh flag = %d\n",plotbufferptr->data[524287]);
          plot->data[524287] = 0;
          if(plotbufferptr->data[524287]==524287) 
            {
            i=0;
            for ( j = rstart+1; j <= rstop; j++) 
                {
                datum = plotbufferptr->data[i+524288];
                plot->data[524288+i++] = datum;
                /* if (datum > plot->max) plot->max = datum; */
                }
            plot->data[524287] = 524287;
            }
          }
notactive:
//        if(shmdt(shmpltptr) == -1)
//          {
//          printf("\nshmdt failed in updatedata.\n");
//          }
#ifdef DEBUGSHM
        printf("\nUPDATEdata detatching %d\n",id);
#endif
/*      if(shmctl(id,IPC_RMID,0) == -1)
          {
          printf("\nshmctl release failed in updatedata.\n");
          }
*/

        return(0);
}
  
                             
/* Taken from Young, pg 141. */
void xs_wprintf(va_alist)
	va_dcl
{
	Widget	w;
        char    *format,*sp,hname[128];
	va_list	args;
	char	str[512];
	Arg	wargs[10];
	XmString	xmstr,xmstr1,xmstr2;
	int	n,rc;
        size_t  hlen;

	va_start(args);

	w = va_arg(args, Widget);	/* get the widget to write to */
	if (!XtIsSubclass( w, xmLabelWidgetClass)) {
		XtError("xs_wprintf() requires a Label Widget");
	}

	format = va_arg(args, char *);

	vsprintf(str, format, args);
	/* xmstr = XmStringLtoRCreate(str, XmSTRING_DEFAULT_CHARSET); */
        hlen = sizeof(hname);
        if ((rc=gethostname(hname,hlen)) < 0 ) {
                strcpy(hname,"UNKNOWN");
        }
        if ((sp = strchr(hname,'.')) != NULL) {
                sp[0] = 0;      /* replace first dot with null */
        }
        strcat(hname,": ");
        xmstr = XmStringCreateLtoR(hname, XmSTRING_DEFAULT_CHARSET);
        xmstr1= XmStringCreateLtoR(str, XmSTRING_DEFAULT_CHARSET);
        xmstr2= XmStringConcat(xmstr,xmstr1);

	n = 0;
	XtSetArg(wargs[n], XmNlabelString, xmstr2); n++;
	XtSetValues( w, wargs, n);

	va_end(args);
}

// #include "graphics.h"
/*  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    R I N P T X   --  wait/check for input reports


    fw = 0  -->  no wait, only check for report
         1  -->  wait on report

    fm = 0  -->  do not consider mouse motion reports
         1  -->         consider mouse motion reports

    fb = 0  -->  do not consider mouse button reports
         1  -->         consider mouse button reports

    fk = 0  -->  do not consider keyboard reports
         1  -->         consider keyboard reports

    x  <--  current x-pixel location of mouse cursor
    y  <--  current y-pixel location of mouse cursor

    b = -1  <--  no     mouse button was pushed
         1  <--  left   mouse button was pushed
         2  <--  middle mouse button was pushed
         3  <--  right  mouse button was pushed

    k = -1  <--  no keyboard key was pushed
      >= 0  <--  keysym of keyboard key that was pushed

    rinptx = 0  <--  requested report did not occur
             1  <--  requested report did occur

*/
int rinptx (fw,fm,fb,fk,x,y,b,k)
int fw,fm,fb,fk,*x,*y,*b,*k;

{
        Display *dsply;
        XEvent report;
	int keycode,index;

        dsply = XtDisplay(canvas);
        *x = *y = *b = *k = -1;
       
        while (1)
        {
                if(fw == 0 && XPending(dsply) == 0) return(0);

                XNextEvent(dsply,&report);

                switch (report.type)
                {
                        case MotionNotify:

                                if(fm == 0) break;

                                *x = report.xmotion.x;
                                *y = report.xmotion.y;

                                return(1);
                   
                        case ButtonPress:

                                if(fb == 0) break;

                                *x = report.xbutton.x;
                                *y = report.xbutton.y;
                                *b = report.xbutton.button;

                                return(1);
                   
                        case KeyPress:

                                if(fk == 0) break;

                                *x      = report.xkey.x;
                                *y      = report.xkey.y;
                                keycode = report.xkey.keycode;
                                index   = report.xkey.state;
                                *k      = XKeycodeToKeysym(dsply,keycode,
                                                index);
                                *k = keycode;
                                return(1);
                   
                }
        }
}
void rflshx ()
{
        Display *dsply;
	dsply = XtDisplay(canvas);
        XFlush(dsply);
}
