/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel                                                */
/*    Created:      Mon Mar 23 2020                                           */
/*    Description:  PID Graph To Help For Tuning (Credits To James Pearman)   */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// PIDMotor             motor         1               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <vector>
using namespace vex;
vex::competition    Competition;
/*-------------------------------Variables-----------------------------------*/
double ColumnTarget;
double ColumnDesired;
double ColumnKP = 0.0;
double ColumnPreviousError = 0.0;
double ColumnDerivative = 0.0;
double ColumnKD = 0.0;
double ColumnIntegral = 0.0;
double ColumnKI = 0.0;
double ColumnError;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void pre_auton( void ) {
    vexcodeInit();
    PIDMotor.setPosition(0, degrees);
}
/*---------------------------------------------------------------------------*/
class graph {
  #define NUM_POINTS  480
  private:
    // class to hold points for a single graph line
    class points {
      public:
        uint32_t         *_points;
        vex::brain::lcd  &_screen;
        vex::color        _color;
        points( vex::brain::lcd &screen ) : _screen(screen) {
          // allocate memory on heap
          _points = new uint32_t[NUM_POINTS];
          // init everything to some value we consider invalid
          for(int i=0;i<NUM_POINTS;i++) {
            _points[i] = INT32_MAX;
          }
          // default line color
          _color = vex::white;
        }
        ~points(){
          // deallocate memory
          delete _points;
        }
        // draw the line
        // There's a variety of ways to do this, could be another property of the class
        void draw() {
          _screen.setPenColor( _color );
          for(int x=0;x<NUM_POINTS-2;x++) {
            if( _points[x] != INT32_MAX ) {
              _screen.drawLine( x, _points[x], x+1, _points[x+1]);
              _screen.drawCircle( x, _points[x], 2, _color );
            }
          }
        }
        // add a point to this line
        void addPoint( int value ) {
          for(int i=0;i<NUM_POINTS-1;i++) {
            _points[i] = _points[i+1];
          }
          _points[NUM_POINTS-1 ] = value;
        }
        // set color for this line
        void setColor( vex::color c ) {
          _color = c;
        }
    };
    public:
      vex::brain _brain;
      std::vector<graph::points *> _points;
      int   _origin_x;
      int   _origin_y;
      graph( int seqnum, int origin_x, int origin_y ) : _origin_x(origin_x), _origin_y(origin_y) {
        // allocate and store each line
        for( int i=0;i<seqnum;i++ ) {
          _points.push_back( new graph::points(_brain.Screen) );
        }
        // thread to render this graph
        thread( render, static_cast<void *>(this) );
      }
      ~graph(){
        // we should deallocate the vector members here really
      }
      // Thread that constantly draws all lines
      static int render(void *arg ) {
        if( arg == NULL)
          return(0);
        graph *instance = static_cast<graph *>(arg);

        while( 1) {
            // this will call render, no need for any other delays
            instance->draw();
        }
        return(0);
      }
      // Draw graph X and Y axis
      void drawAxis() {
        _brain.Screen.setPenColor( vex::white );
        _brain.Screen.drawLine( _origin_x, 0, _origin_x, 240 );
        _brain.Screen.drawLine( 0, _origin_y, 480, _origin_y );
        for( int x=0;x<=480;x+=20 ) {
          _brain.Screen.drawLine( x, _origin_y+5, x, _origin_y-5 );
        }
        for( int y=0;y<=240;y+=20 ) {
          _brain.Screen.drawLine( _origin_x+5, y, _origin_x-5, y );
        }
      }
      // draw everything
      void draw() {
        _brain.Screen.clearScreen( vex::color(0x202020) );
        drawAxis();
        for(int id=0;id<_points.size();id++)
          _points[id]->draw();
        _brain.Screen.render();
      }
      // add a point to a particular sequence
      void addPoint( int id, int value ) {
        if( id < _points.size() )
          _points[id]->addPoint(value + _origin_y );
      }
      // set the color of this sequence
      void setColor( int id, vex::color c ) {
        if( id < _points.size() )
          _points[id]->setColor( c );
      }
};
int ActualValueTask( void *arg ) {
    if( arg == NULL ) return 0;
    graph *g = static_cast<graph *>(arg);
    int motordegree = ((PIDMotor.rotation(rotationUnits::deg)/4));
    while(1) {
      g->addPoint( 0, motordegree);
      this_thread::sleep_for(15);
    }
}
int TargetValueTask( void *arg ) {
    if( arg == NULL ) return 0;
    graph *g = static_cast<graph *>(arg);
    while(1) {
      g->addPoint( 1, ColumnDesired/4 );
      this_thread::sleep_for(15);
    }
}
/*----------------------------------------------------------------------------*/
/*                              User Control Task                             */
/*----------------------------------------------------------------------------*/
void usercontrol( void ) {
  while (1){
    if(Controller1.ButtonB.pressing()) { //Tray Forward Button
        ColumnDesired = 800; //The desired value changes and the kP value is tuned to our requirement
        ColumnKP = 0.0093; //Tune This To See A Difference In The Graph
        ColumnKD = 0.00;        //Tune This To See A Difference In The Graph
        ColumnKI = 0.0000000;   //Tune This To See A Difference In The Graph
    }
    else if(Controller1.ButtonA.pressing()) { //Tray Backward Button
        ColumnDesired = 0; //The desired value changes and the kP value is tuned to our requirement
        ColumnKP = 0.0093;
        ColumnKD = 0;
        ColumnKI = 0;
    }
    else {
        ColumnDesired = PIDMotor.rotation(rotationUnits::deg); //The current position is desired so it doesn't move
        ColumnKP = 0;
        ColumnKD = 0;
        ColumnKI = 0;
        PIDMotor.stop(vex::brakeType::coast); //This is a backup stop just in case
    }
    ColumnError = pow((PIDMotor.rotation(rotationUnits::deg) - ColumnDesired),3); //This calculates the error from desired value and current value
    ColumnDerivative = ColumnError - ColumnPreviousError; //This find the difference between the current error and previous error 
    if (((PIDMotor.rotation(rotationUnits::deg) - ColumnDesired) < 5.0) && ((PIDMotor.rotation(rotationUnits::deg) - ColumnDesired) > -5.0)){
        ColumnIntegral = 0; //We don't need integral if the error is within +/- 5
    }
    else { //If It isn't within +/- 5
        ColumnIntegral += ColumnError; 
    }
    PIDMotor.spin(reverse,(ColumnError * ColumnKP + ColumnDerivative * ColumnKD + ColumnIntegral * ColumnKI), voltageUnits::volt); //This calculates the voltage needed for the tray to move
    // 4 lines, axis at position 40, 240 
    graph g( 4, 20, 220 );
    // set line colors
    g.setColor(0, vex::color::green );
    g.setColor(1, vex::color::red );
    // and we are using separate tasks to add points to each line, this is a bit overkill
    thread t2( ActualValueTask, static_cast<void *>(&g) );
    thread t1( TargetValueTask, static_cast<void *>(&g) );
    Brain.Screen.setFont(vex::fontType::mono15);
    Brain.Screen.printAt(100, 272-20, "Actual: %f",ActualValueTask);
    Brain.Screen.printAt(250, 272-20, "Target: %f",TargetValueTask);
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering
    ColumnPreviousError = ColumnError; //Updates the variables
    vex::task::sleep(15); //Slight delay so the Brain doesn't overprocess
  }
}
int main() {
    pre_auton();
    Competition.drivercontrol( usercontrol ); //Calls the driver control function
    while(1) {
      vex::task::sleep(15); //Slight delay so the Brain doesn't overprocess
    }
}