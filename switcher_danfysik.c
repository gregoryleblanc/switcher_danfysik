
/*----------------------------------------------------------------------------
 *  Copyright (c) 1991 - 2022              Ohio University
 *                                         Edwards Accelerator Laboratory
 *
 *  This software was partially developed under a United States Government
 *  license described in the Notice file included as part of this distribution
 * 
 *    Ohio University Accelerator Laboratory, Athens, Ohio  45701
 * 
 *    carter@ohio.edu   Tel: (614) 593-1984   
 *----------------------------------------------------------------------------
 *
 * Authors:
 *
 *	Don Carter
 *  John O'Donnell
 *
 *	Ohio University Accelerator Laboratory
 *
 * Original version DAQ created 1991
 * Revision History:
 *   Revised for Danfysik switcher power supply 10 November 2022  D.E.Carter
 *
 *   This program is used to control the switcher magnet current 
 *   using mouse on the console computer.
 *   This version communicates with the Danfysik System 8500 power supply
 *   using the serial port /dev/ttyUSB0 and sends commands to adjust the 
 *   magnet current.
 *----------------------------------------------------------------------------*/

#define _GNU_SOURCE  // needed to disable CRTSCTS for serial port

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <signal.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "oual.h"
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
#include <X11/XKBlib.h>

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
struct sockaddr_in s_in;
struct sockaddr_in p_in;
struct hostent *hp;

float xval=0.001,yval=6.0,txval,tyval,save_xval=0.001,save_yval=6.0;
// float xvalmax=13.7; // calibration constant for switcher max ppm
float xvalmax=13.57396; // calibration factor for Danfysik power supply

//

void FineCB();
void CoarseCB();
void SaveCB();
void RestoreCB();
void LockCB();
void QuitCB();
void RedrawCB();
void ResizeCB();
void xs_wprintf(Widget wid, char *fmt, ...);
void TimeOutCB();
int  readdata();
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

char *shmpltptr;
struct plotbuffer *plotbufferptr;

	Widget  canvas;
 
int main(argc, argv)
	int argc;
	char *argv[];
{
	Widget 	toplevel, frame, infobar, quitbutton; //ex1, ex2, ex3, ex4, home,
	Widget  fine, coarse, save, restore, lock; //yup, ydn, yauto
        rubber_band_data data;
	XtAppContext app;
	XSetWindowAttributes	attrs;
	XmString	xmstr;
	Arg	wargs[10];
	extern GC	SimpleGC;
	XGCValues	gcvalues;
	int	n;
// 	int  	j;
	unsigned long	valuemask;
//	char command[100];

//        char xvalstring[21];

	toplevel = XtVaAppInitialize(&app, "Simple", NULL, 0, 
				&argc, argv, NULL, NULL);

	/* XtInitialize is called first to let to digest any command
		line arguments for windows; it will leave anything it
		doesn't want for us. */
 
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

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {
        printf("Could not open port /dev/ttyUSB0\n");
        return 1;
    }

// initialize /dev/ttyUSB0 parameters

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

    printf("Opened successfully /dev/ttyUSB0\n");

    sleep(1);

	n = 0;
	XtSetArg(wargs[n], XmNheight, 480); n++;
	XtSetArg(wargs[n], XmNwidth, 640); n++;
        XtSetArg(wargs[n], XmNx    , 640); n++;
        XtSetArg(wargs[n], XmNy    , 480); n++;
	frame = XtCreateManagedWidget("Frame", xmFormWidgetClass,
				toplevel, wargs, n);

	n = 0;
	XtSetArg(wargs[n], XmNleftAttachment, XmATTACH_FORM); n++;
	XtSetArg(wargs[n], XmNrightAttachment, XmATTACH_FORM); n++;
	XtSetArg(wargs[n], XmNtopAttachment, XmATTACH_FORM); n++;
	XtSetArg(wargs[n], XmNorientation, XmHORIZONTAL); n++;
	infobar = XmCreateRowColumn(frame, "infobar", wargs, n);
	XtManageChild(infobar);

// define the "Quit" button
	n = 0;
	xmstr = XmStringCreate("Quit", XmSTRING_DEFAULT_CHARSET);
	XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
	XtSetArg(wargs[n], XmNadjustLast, False); n++;
	XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        //XtSetArg(wargs[n], XmNbackground, 0xf08080); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*255+256*211+155); n++; // burlywood
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
	quitbutton = XmCreatePushButton(infobar, "quitbutton", wargs, n);
	XtAddCallback(quitbutton, XmNarmCallback, QuitCB, (XtPointer) 1);
	XtAddCallback(quitbutton, XmNactivateCallback, QuitCB, (XtPointer) 2);
	XtAddCallback(quitbutton, XmNdisarmCallback, QuitCB, (XtPointer) 3);
	XtManageChild(quitbutton);
  
// define the "Fine" button
        n = 0;
        xmstr = XmStringCreate("Fine", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        //XtSetArg(wargs[n], XmNbackground, 0xffb6c1); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*255+256*236+139); n++; // lightgoldenrod
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        fine = XmCreatePushButton(infobar, "fine", wargs, n);
        XtAddCallback(fine, XmNarmCallback, FineCB, (XtPointer) 1);
        XtAddCallback(fine, XmNactivateCallback, FineCB, (XtPointer) 2);
        XtAddCallback(fine, XmNdisarmCallback, FineCB, (XtPointer) 3);
        XtManageChild(fine);

// define the "Coarse" button
        n = 0;
        xmstr = XmStringCreate("Coarse", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 0x00ee76); n++;
        XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        coarse = XmCreatePushButton(infobar, "coarse", wargs, n);
        XtAddCallback(coarse, XmNarmCallback, CoarseCB, (XtPointer) 1);
        XtAddCallback(coarse, XmNactivateCallback, CoarseCB, (XtPointer) 2);
        XtAddCallback(coarse, XmNdisarmCallback, CoarseCB, (XtPointer) 3);
        XtManageChild(coarse);
  
// define the "Save" button
        n = 0;
        xmstr = XmStringCreate("Save", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        //XtSetArg(wargs[n], XmNbackground, 0xffb6c1); n++;
        //XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*255+256*0+255); n++; // magenta
        XtSetArg(wargs[n], XmNforeground, 0xffffff); n++;
        save = XmCreatePushButton(infobar, "save", wargs, n);
        XtAddCallback(save, XmNarmCallback, SaveCB, (XtPointer) 1);
        XtAddCallback(save, XmNactivateCallback, SaveCB, (XtPointer) 2);
        XtAddCallback(save, XmNdisarmCallback, SaveCB, (XtPointer) 3);
        XtManageChild(save);

// define the "Restore" button
        n = 0;
        xmstr = XmStringCreate("Restore", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        //XtSetArg(wargs[n], XmNbackground, 0x00ee76); n++;
        //XtSetArg(wargs[n], XmNforeground, 0x000000); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*230+256*99+99); n++; //indianred1
        XtSetArg(wargs[n], XmNforeground, 0xffffff); n++;
        restore = XmCreatePushButton(infobar, "restore", wargs, n);
        XtAddCallback(restore, XmNarmCallback, RestoreCB, (XtPointer) 1);
        XtAddCallback(restore, XmNactivateCallback, RestoreCB, (XtPointer) 2);
        XtAddCallback(restore, XmNdisarmCallback, RestoreCB, (XtPointer) 3);
        XtManageChild(restore);
                            
// define the "Lock" button
        n = 0;
        xmstr = XmStringCreate("Lock", XmSTRING_DEFAULT_CHARSET);
        XtSetArg(wargs[n], XmNlabelString, xmstr); n++;
        XtSetArg(wargs[n], XmNadjustLast, False); n++;
        XtSetArg(wargs[n], XmNpacking, XmPACK_NONE); n++;
        XtSetArg(wargs[n], XmNbackground, 256*256*255+256*20+147); n++; //deeppink
        XtSetArg(wargs[n], XmNforeground, 0xffffff); n++;
        lock = XmCreatePushButton(infobar, "lock", wargs, n);
        XtAddCallback(lock, XmNarmCallback, LockCB, (XtPointer) 1);
        XtAddCallback(lock, XmNactivateCallback, LockCB, (XtPointer) 2);
        XtAddCallback(lock, XmNdisarmCallback, LockCB, (XtPointer) 3);
        XtManageChild(lock);    

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
	XtAddCallback(canvas, XmNresizeCallback, ResizeCB, (XtPointer) 1);
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


// section to follow mouse while updating the magnet value

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
//char y_command[100];
int dispx,dispy;

if(magnet_lock == 1) return;

if(halt_update == 1)
  {
  rubberband=1;
  current_x = event->xbutton.x;
  current_y = event->xbutton.y;

  time=50;

  yval=6.0000;
  
//
// highres indicates the sensitivity of the mouse current movements
//   highres = 0:  Grab mode, Red color, current follows mouse (fastest movement)
//   highres = 1:  Medium mode, Green color, medium current movements
//   highres = 2:  Fine mode, Blue color, fine current movements
//   highres = 3:  ExtraFine mode, Magenta color, extra fine movements
//   highres = 4:  SuperFine mode, Black color, super fine movements
//   highres = 5:  Only used for quadrupoles, Cyan color, dA = dB movements
//   highres = 6:  Only used for quadrupoles, Gray color, A or B only movements
//

    if(highres == 0) {
    txval=((float)current_x/(float)new_width)*XSCALE;
    tyval=((float)((new_height-20.0-current_y))/(float)(new_height-36.0))*YSCALE;
    if(fabs(txval - xval) < .5) { 
         xval=txval;
	 }
    }
  if(highres == 1) {
    if(current_x > data->last_x)xval=xval+.004;
    if(current_x < data->last_x)xval=xval-.004;
    }
  if(highres == 2) {
    if(current_x > data->last_x)xval=xval+.001;
    if(current_x < data->last_x)xval=xval-.001;
    }

  if(highres == 3) {
    if(current_x > data->last_x)xval=xval+.00025;
    if(current_x < data->last_x)xval=xval-.00025;
    }
  if(highres == 4) {
    if(current_x > data->last_x) xval=xval+.00005;
    if(current_x < data->last_x) xval=xval-.00005;
    }
   
  if(highres == 5) {
    if(current_x > data->last_x) {
       xval=xval+.004;
       }
    if(current_x < data->last_x) {
       xval=xval-.004;
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
  if(xval < 0.001) xval = 0.001;
  
  
  ixval = (xval/xvalmax)*1000000;
  // printf("ixval = %06d\n",ixval);
  sprintf(x_command,"da 0 %06ld",ixval);

// write messages to the Danfysik                    
 
write(fd, x_command, strlen(x_command));
write(fd, "\n", 1);
	  
  data->last_x = event->xbutton.x;
  data->last_y = event->xbutton.y;
  }
}

// section to increment as a function of button pressed and released...
// button 1 (left) = decrement the magnet value 1 step
// button 2 (middle) =  cycle through the resolutions (finer each press looping at end)
// button 3 (right) = increment the magnet value 1 step

void end_rubber_band(w,data,event)
Widget    w;
rubber_band_data  *data;
XEvent            *event;
{
//int start,stop,len,j;
char x_command[100];
//char y_command[100];

if(magnet_lock == 1) return;

//
// highres indicates the sensitivity of the mouse current movements
//   highres = 0:  Grab mode, Red color, current follows mouse (fastest movement)
//   highres = 1:  Medium mode, Green color, medium current movements
//   highres = 2:  Fine mode, Blue color, fine current movements
//   highres = 3:  ExtraFine mode, Magenta color, extra fine movements
//   highres = 4:  SuperFine mode, Black color, super fine movements
//   highres = 5:  Only used for quadrupoles, Cyan color, dA = dB movements
//   highres = 6:  Only used for quadrupoles, Gray color, A or B only movements
//

if(rubberband  == 0) {
  if(event->xbutton.button == Button1){
    if(highres == 0) xval = xval-.016;
    else if(highres == 1) xval=xval-.004;
    else if(highres == 2) xval=xval-.001;
    else if(highres == 3) xval=xval-.00025;
    else if(highres == 4) xval=xval-.00005;
    else if(highres == 5) xval=xval-.004;
    else if(highres == 6) xval=xval-.004;
    }
  if(event->xbutton.button == Button3){
    if(highres == 0) xval = xval+.016;
    else if(highres == 1) xval=xval+.004;
    else if(highres == 2) xval=xval+.001;
    else if(highres == 3) xval=xval+.00025;
    else if(highres == 4) xval=xval+.00005;
    else if(highres == 5) xval=xval+.004;
    else if(highres == 6) xval=xval+.004;
    }
  if(event->xbutton.button == Button2){
    if(highres == 0) highres = 1;
    else if(highres == 1) highres = 2;
    else if(highres == 2) highres = 3;
    else if(highres == 3) highres = 4;
    else if(highres == 4) highres = 0;
    }
  }
ixval = (xval/xvalmax)*1000000;
// printf("ixval = %06d\n",ixval);
sprintf(x_command,"da 0 %06ld",ixval);
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

void enter_window(w,event)
	Widget    w;
//	focus_change_data *data;
	XEnterWindowEvent   *event;
{
	//printf("\nevent focus change type=%d mode=%d detail=%d same_screen=%d focus=%d\n",event->type,event->mode,event ->detail,event->same_screen, event->focus);

}

void exit_window(w,event)
	        Widget    w;
		//focus_change_data *data;
		XLeaveWindowEvent   *event;
{

	//printf("\nevent focus change type=%d mode=%d detail=%d same_screen=%d focus=%d\n",event->type,event->mode,event ->detail,event->same_screen, event->focus);
	//printf("\nevent focus change type=%d \n",event->type);		
}

void forcedexit()
  {
//waittilldone:
  close(fd);
  exit(0);
  }



void QuitCB(w, client_data, call_data)
        Widget  w;
        struct dataplot *client_data;
        XmDrawingAreaCallbackStruct *call_data;
{
//        XColor  col, unused;
//        Colormap cmap = DefaultColormapOfScreen (XtScreen (w));

        extern GC       SimpleGC;
        char strbuff[100];
        Display *dsply;
//        Window drawable;
        dsply = XtDisplay(w);
//        drawable = XtWindow(w);

        static int flag;
        int k = 0;
//        int x, y, b, z;
        int i;
        i = (long) client_data;


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
//                        z=rinptx (1, 0, 0, 1, &x, &y, &b, &k);
                        //printf("\n%d %d %d %d %d",x,y,b,k,z);
                        if(k!=29){
                            printf("\nExpecting 29 but found %i",k);
//                            return 0;
                            }
                        XtCloseDisplay(XtDisplay(w));
                        close(fd);
                        exit(0);
                }

        }
}

// 
// FineCB and CoarseCB used to increase or decrease the sensitivity of the current movements
// with respect to the mouse movements and button presses.
//
// highres indicates the sensitivity of the mouse current movements
//   highres = 0:  Grab mode, Red color, current follows mouse (fastest movement)
//   highres = 1:  Medium mode, Green color, medium current movements
//   highres = 2:  Fine mode, Blue color, fine current movements
//   highres = 3:  ExtraFine mode, Magenta color, extra fine movements
//   highres = 4:  SuperFine mode, Black color, super fine movements
//   highres = 5:  Only used for quadrupoles, Cyan color, dA = dB movements
//   highres = 6:  Only used for quadrupoles, Gray color, A or B only movements
//

void FineCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
//	int delta;
        i = (long) client_data;

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
	           if(highres == 0) highres=1;
		   else if(highres == 1) highres=2;
		   else if(highres == 2) highres=3;
		   else if(highres == 3) highres=4;
		   else if(highres == 5) highres=1;
		   else if(highres == 6) highres=1;
                }

        }
}

void CoarseCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
//        int delta;
        i = (long) client_data;

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
                   if(highres == 4) highres=3;
		   else if(highres == 3) highres=2;
		   else if(highres == 2) highres=1;
		   else if(highres == 1) highres=0;
		   else if(highres == 5) highres=0;
		   else if(highres == 6) highres=0;
                }

        }
}
  
// SaveCB is used to remenber the current current setting so it can be restored by the "Restore" button

void SaveCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
//        int delta;
        i = (long) client_data;

        switch(i) {

        case 1: /* Arm */
                flag = 0;       /* if we disarm now, nothing happens */
                break;
        case 2: /* Activate */
                flag = 1;       /* user released button, stand by to quit */
                break;
        case 3: /* Disarm */
                if (flag == 1) {        /* finished with things, now exit */
		   save_xval = xval;
		   save_yval = yval;
                }

        }
}

void RestoreCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
        FILE * input;
//        int input;
//        int delta;
        i = (long) client_data;

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
		   xval = save_xval;
		   yval = save_yval;
                   if ((input = fopen("switcher.tandem.data","w")) != NULL) {
                      fprintf(input,"%s %d %f %f\n","switcher",1,xval*SHUNT_FACTOR,yval);
                      fclose (input);
                      }
                }

        }
}

// Lock button is used to prevent movement of the magnet current until it is "Unlocked"
// by pressing the "Lock" button and toggeling "Lock" mode off  

void LockCB(w, client_data, call_data)
        Widget  w;
        int     *client_data;
        XmAnyCallbackStruct     *call_data;
{
        static int flag;
        int i;
//        int delta;
        i = (long) client_data;

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
	int j,flag;
    FILE * input;
	float new_xval, new_yval;
	char device_name[100];
	char x_command[100];
//	char y_command[100];
	
	new_xval = xval;
	new_yval = yval;
	
        if((refloops > 0) && (mousemoved == 0))
          {
          if(--refloops == 0)
            {
            if(shmctl(id,IPC_RMID,0) == -1)
              {
              printf("\nshmctl release failed in QUIT.\n");
              }
            exit(0);
            }
          }
          if(XtIsRealized(canvas))
           XClearArea(XtDisplay(canvas),XtWindow(canvas),0,0,0,0,TRUE);

     if(plot.autoy == 1) {
        plot.max = 16;
        for (j = plot.start; j <= plot.stop; j++) {
                while (plot.data[j] > plot.max) plot.max = plot.max << 1;
                }
        }

//
// switcher.tandem.data is a file that is used to set the switcher magnet to a specific value.
//
// the format of the file is "%s %d %f %f",device_name,&flag,&new_xval,&new_yval
//   device name = "switcher" for this device
//   flag = 1 to set the switcher to a new value, reset to 0 after it has been set
//   xval = value to set the magnet current to
//   yval = not used for switcher, only for quadrupoles
//
	if (rubberband == 1) time = 050;
	else { 
             time=500;
	     if ((input = fopen("switcher.tandem.data","r")) != NULL) {
	        fscanf(input,"%s %d %f %f",device_name,&flag,&new_xval,&new_yval);
		//printf("\n%s %d %f %f",device_name,flag,new_xval,new_yval);
		fclose (input);
	        }
	     if(flag == 1) {
                xval = new_xval/SHUNT_FACTOR;
                ixval = (xval/xvalmax)*1000000;
                // printf("ixval = %06d\n",ixval);
                sprintf(x_command,"da 0 %06ld",ixval);

	         // write messages to the Danfysik                    
		
		write(fd, x_command, strlen(x_command));
		write(fd, "\n", 1);

	        }
	     if ((input = fopen("switcher.tandem.data","w")) != NULL) {
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
	int x, y, oldx, oldy, i, j, sum, z, dz;//width,
        int slx, sly, sli, slj, slwidth, sly0;//, slx0, temp;
//        int rx,ry,rb,rk;
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
        i = 0; 
	if(rtype == 0)
            {
            for (j = plot.start; j <= plot.stop; j++) 
		{
                sum += plot.data[j];
                if(plot.max > 8)
                   y = (new_height - 20)/2;
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
//
// highres indicates the sensitivity of the mouse current movements
//   highres = 0:  Grab mode, Red color, current follows mouse (fastest movement)
//   highres = 1:  Medium mode, Green color, medium current movements
//   highres = 2:  Fine mode, Blue color, fine current movements
//   highres = 3:  ExtraFine mode, Magenta color, extra fine movements
//   highres = 4:  SuperFine mode, Black color, super fine movements
//   highres = 5:  Only used for quadrupoles, Cyan color, dA = dB movements
//   highres = 6:  Only used for quadrupoles, Gray color, A or B only movements
//  
			    
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
                x = (ix * xstep)+(float)new_width*0.05;
                y = (((float)new_height*0.9) - ((j/64) * ystep));
                z=((float)plot.data[j] / (float)plot.max)*XSCALE;
                if(z > 15) z = 15;
                if(plot.data[j] > 0)
                    {
                    if(z > 0) {
                        XSetForeground(dsply,SimpleGC,ouZcolor[z]);
                        XFillRectangle(dsply,drawable,SimpleGC,x,y,xstep*0.8,ystep*0.8);
                    }

                    if ((ix==curmouse_x) && (iy==curmouse_y))
                       {
                       XSetForeground(dsply,SimpleGC,ouWhite);
                       XDrawRectangle(dsply,drawable,SimpleGC,
                            x-(0.1*xstep), y-(0.1*ystep),xstep,ystep);
                       slxstep = (float)new_width*0.24;
                       slxstep = slxstep / 64;
                       slystep = (new_height - 20)/1.5;
                       slystep = slystep / (2.0*plot.max);
                       sli=0;
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
                                XSetForeground(dsply,SimpleGC,ouYellow);
                                XDrawLine(dsply,drawable,SimpleGC,slx,sly-5,slx,sly+5);
                                XDrawLine(dsply,drawable,SimpleGC,slx-5,sly,slx+5,sly);
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
                                XSetForeground(dsply,SimpleGC,ouWhite);
                       sprintf(strbuff,"Z Max = %d",plot.max);
                       XDrawString(dsply,drawable,SimpleGC,
                            new_width,0.1*sly0,strbuff,strlen(strbuff));
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
                                XSetForeground(dsply,SimpleGC,ouWhite);
                                XDrawLine(dsply,drawable,SimpleGC,slx,sly-5,slx,sly+5);
                                XDrawLine(dsply,drawable,SimpleGC,slx-5,sly,slx+5,sly);
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
                    }
                i++;
		oldx = x;
		oldy = y;
		}
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
            thresh=((float)plot.max*0.01)+1;
            ystep=ystep/2.0;
            XSetForeground(dsply,SimpleGC,ouWhite);
            xstep = ((float)new_width*0.9)/128.0;
            for (j = plot.start; j <= plot.stop; j++)
                {
                iy = j/64;
                ix = j-(64*iy);
                iz = plot.data[j];
                //x = ((j-(64*(j/64))) * xstep)+(float)new_width*0.05;
                x = ((ix * xstep)+(float)new_width*0.05)+((float)iy*xstep);
                
                
                sum += iz;
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
                oldx = x;
                oldy = y;
                }
            }
  mouse_channel=(xval/XSCALE)*(float)new_width;
  mouse_value=(yval/YSCALE)*(float)(new_height-36);
  if(rtype == 0)
    {
    if(new_width > size_thresh)
      {
      sprintf(strbuff,"0");
      XDrawString(dsply,drawable,SimpleGC,2,new_height-5,strbuff,strlen(strbuff));
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
    sprintf(strbuff,"Switcher Magnet Current = %7.4f Amps",SHUNT_FACTOR*xval);
    XDrawString(dsply,drawable,SimpleGC,10,12,strbuff,strlen(strbuff));
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
//   String color = (String) client_data;
   Display *dpy = XtDisplay (w);
//   Colormap cmap = DefaultColormapOfScreen (XtScreen (w));
   XColor  col; //, unused;
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
//	char 	fname[128];
    int rstart, rstop;
//	int	rdisk,rtag,rstart,rstop,rdate,rtime;
	int	i,j;//,datum;

#ifdef DEBUGSHM
        printf("\nREADdata attaching %d\n",id);
#endif
	rstart=0;
	rstop=1000;
	rtype=0;
	if(rtype != 1) rtype = 0;
        i = (int)rstart;
	//datum=plotbufferptr->data[i];
	plot->min = 0;
	plot->max = 1000;
	plot->data[i++] = 0;
	for ( j = rstart+1; j <= rstop; j++) {
		//datum = plotbufferptr->data[i];    
		plot->data[i++] = j;
	}
	plot->start = rstart;
        plot->istart = rstart;
	plot->stop = rstop;
        plot->istop = rstop;
	plot->autoy = 1;
	return(0);
}


/* Taken from Young, pg 141. */
//void xs_wprintf(va_alist)
void xs_wprintf(Widget wid, char *fmt, ...)
//	va_dcl
{
//	Widget	w;
        char    *format,*sp,hname[128];
	va_list	args;
	char	str[512];
	Arg	wargs[10];
	XmString	xmstr,xmstr1,xmstr2;
	int	n,rc;
        size_t  hlen;

	va_start(args, fmt);

//	w = va_arg(args, Widget);	/* get the widget to write to */
	if (!XtIsSubclass( wid, xmLabelWidgetClass)) {
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
	XtSetValues( wid, wargs, n);

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
                                *k      = XkbKeycodeToKeysym(dsply,keycode, 0,
                                                index);
                                *k = keycode;
//                                return(1);
                   
                }
        }
}

void rflshx ()
{
        Display *dsply;
	dsply = XtDisplay(canvas);
        XFlush(dsply);
}
