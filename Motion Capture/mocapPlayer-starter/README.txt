Name: Joe Yu-Ho Chang
OS: Visual Studio 2017, fltk 1.3.2
Note: The debugging working directory for both solotuions are set to '..\..\..\mocapPlayer-starter'

Graphs:
Individual graphs are in Graphs folder (JPG format).

Extra Credits:
1. Make the OpenGL renderer prettier.
   -> Made window wider (741 -> 750)to have right margins to show whole buttons on the right for Windows (741 is fine on OSX)
      interface.cpp line 52
   -> Added YELLOW and PURPLE for joint colors. Used for load 5 kinds of skeletons at the same time.
      displaySkeleton.h line 29, displaySkeleton.cpp line 24-25
   -> Changed skeletonID from Input to Dropdown Menu
      interface.h line 19, 75-76, interface.cpp line 164-168, mocapPlayer.cpp line 278-281, 650-658
   -> Format the About Message Box
      mocapPlayer.cpp line 763
   -> Added Reset Camera Button
      interface.h line 41-42, interface.cpp line 80-83, mocapPlayer.cpp line 284-298
   -> Adjusted width of 5 buttons below screen and slider
   -> Adjusted position of buttons below slider

2. Analyze the computation time of the different interpolation techniques. (Done)
   -> Inside report section IV. 

3. Support keyframes that are non-uniform in time. (Done)
   -> Added an argument for tolerance to cause random non-unifrom keyframes
      (e.g. 135-martialArts.asf 135_06-martialArts.amc b q 40 135_06-martialArts-bq-N40-T5.amc 5)
      Here, T (tolerance) is 5.
      Now the interval between each keyframe is N', where N - T <= N' <= N + T.
      If the T is not used in arguments, T = 0.



FLTK Setup Note:
On Windows and Linux, please follow the corresponding readme files in the folder: fltk-1.3.2
Note: On Linux, you may need to install the "autoconf" tool.

On Max OS X, you should use fltk-1.3.5. Therefore, follow the instructions in the fltk-1.3.5 folder. 
Also, you need to modify two Makefiles:
%homeworkFolder%/mocapPlayer-starter/Makefile
%homeworkFolder%/mocapPlayer-starter/Makefile.FLTK
Open each file in a text editor, and change fltk-1.3.2 to fltk-1.3.5 .