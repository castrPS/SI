

/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2015 Adept Technology, Inc.
Copyright (C) 2016 Omron Adept Technologies, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
#include "Aria.h"
#include <fstream>

/** @example gotoActionExample.cpp Uses ArActionGoto to drive the robot in a square
 
  This program will make the robot drive in a 2.5x2.5 meter square by
  setting each corner in turn as the goal for an ArActionGoto action.
  It also uses speed limiting actions to avoid collisions. After some 
  time, it cancels the goal (and the robot stops due to a stopping action) 
  and exits.

  Press escape to shut down Aria and exit.
*/


#define barreira INT_MAX
#define mapax 24280
#define mapay 51680
#define gridsize 510
#define PI 3.14159265
#define gridx floor(mapax/gridsize)
#define gridy floor(mapay/gridsize)

/*int mapa[floor(gridx)][floor(gridy)];
double largura, altura;//dimensoes de cada quadrado do grid
double initx, inity, angle;//angulo e posicoes iniciais
double finalx, finaly;//posicao final
double atualx, atualy, atualAngle;//variaveis correspondentes às posições atuais e angulo atual
double d;
void read_file(string file){
  arq = file+".txt"; //ver o nome do arquivo de entrada
  const char* input = arq.c_str();
  FILE * file;
  file = fopen(input, "r");
  rewind(file);
  fscanf(file, "%f %f %f", &initx, &inity, &angle);
  fscanf(file, "%f %f", &finalx, &finaly);
  atualx = initx;
  atualy = inity;
  atualAngle = angle;
  fclose(file);
}

void mapa_init(){
    //iniciando posições não visitadas
    emset(mapa, -1, sizeof(mapa[0][0]) * gridx * gridy);
    //iniciando destino
    mapa[floor(finalx/gridsize)][floor(finaly/gridsize)] = 0;
    mapa[floor(initx/gridsize)][floor(inity/gridsize)] = heuristica();
}*/

void getSonar(ArRobot *thisRobot, int x, int y)
{

int numSonar;
int i;
//Number of sonar on the robot
//Counter for looping
numSonar = thisRobot->getNumSonar(); //Get number of sonar
ArSensorReading* sonarReading;
//To hold each reading
for (i = 0; i < numSonar; i++)
//Loop through sonar
{
sonarReading = thisRobot->getSonarReading(i);
//Get each sonar reading
  double rad= PI / 180.0;
  double angulo= rad*sonarReading->getSensorTh();
  int fx=x+(cos(angulo)*sonarReading->getRange());
  int fy=y+(sin(angulo)*sonarReading->getRange());
  /*mapa[floor(fx/gridsize)][floor(fy/gridsize)] = 0;*/


}
}

/*double heuristica(){
  double x = 0;
  return x;
}
void a-star(){

}*/
int main(int argc, char **argv)
{
  
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArAnalogGyro gyro(&robot);
  ArSonarDevice sonar;
  ArRobotConnector robotConnector(&parser, &robot);
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

  
  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "gotoActionExample: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        // -help not given
        Aria::logOptions();
        Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  ArLog::log(ArLog::Normal, "gotoActionExample: Connected to robot.");

  robot.addRangeDevice(&sonar);
  robot.runAsync(true);

  // Make a key handler, so that escape will shut down the program
  // cleanly
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);
  printf("You may press escape to exit\n");

  // Collision avoidance actions at higher priority
  ArActionLimiterForwards limiterAction("speed limiter near", 300, 600, 250);
  ArActionLimiterForwards limiterFarAction("speed limiter far", 300, 1100, 400);
  ArActionLimiterTableSensor tableLimiterAction;
  robot.addAction(&tableLimiterAction, 100);
  robot.addAction(&limiterAction, 95);
  robot.addAction(&limiterFarAction, 90);

  // Goto action at lower priority
  ArActionGoto gotoPoseAction("goto");
  robot.addAction(&gotoPoseAction, 50);
  
  // Stop action at lower priority, so the robot stops if it has no goal
  ArActionStop stopAction("stop");
  robot.addAction(&stopAction, 40);


  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);

  const int duration = 30000; //msec
  bool first = true;
  int goalNum = 0;
  ArTime start;
  start.setToNow();

  while (Aria::getRunning()) {

  }
  
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;
}