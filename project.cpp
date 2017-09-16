#include <vector>
#include <math.h>
#include <limits.h>
#include "Aria.h"
#include <fstream>

using namespace std;

#define barreira INT_MAX
#define mapax 24285
#define mapay 51685
#define gridsize 510
#define PI 3.14159265
#define gridx floor(mapax/gridsize)
#define gridy floor(mapay/gridsize)
int finaly, finalx;
mapa *map;

class grid {
  public:
    char rep=' '; //representação em char do mapa
    int heur=-1;//valor da heurisitca usado;
    int x;
    int y;
};

//mapa
class mapa{
  public:
    vector< vector<grid> > pos;
};

//leitura de arquivo e inicialização do mapa e posição do robô
void newMap(){
  for(int i = 0; i < mapay; i++){
    for(int j = 0; j < mapax, j++){
      map = new grid();
    }

  }
  map.pos[finaly][finalx].heur = 0;
}

//maeando atráves do sonar
void sonarRound(ArRobot *thisRobot, int x, int y, int th, mapa *map)
{

int numSonar;
int i;
//Number of sonar on the robot
//Counter for looping
numSonar = thisRobot->getNumSonar(); //Get number of sonar
ArSensorReading* sonarReading;
//To hold each reading
for (i = 0; i < numSonar; i++){
  sonarReading = thisRobot->getSonarReading(i);
  sonarReading->resetSensorPosition(x,y,th);
  //achando o angulo
  double rad= PI / 180.0;
  double angulo= rad*sonarReading->getSensorTh();
  //achando a posição no grid 
  int fx=x+(cos(angulo)*sonarReading->getRange());
  fx=floor(fx/gridsize);
  int fy=y+(sin(angulo)*sonarReading->getRange());
  fx=floor(fy/gridsize);
  //Só marca como parede se for menor que o limite do sonar
    if(sonarReading->getRange()<5000){
      //tratamentos para poder caber no mapa
      if(map->pos.size()<=fy){
        map->pos.resize(fy*2+1);
      }
      if(map->pos[0].size()<=fx){
        for(int j=0; j<map->pos.size(); j++){
          map->pos[j].resize(fx*2+1);
        }
      }
      map->pos[fy][fx].rep='#';
      map->pos[fy][fx].heur=INT_MAX;
      map->pos[fy][fx].x=x+(cos(angulo)*sonarReading->getRange());
      map->pos[fy][fx].x= y+(sin(angulo)*sonarReading->getRange());
  }
  }
}


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

  bool first = true;
  int goalNum = 0;
  ArTime start;
  start.setToNow();

  //inicialização
  int x, y, th;
  mapa map;

  while (Aria::getRunning()) {
    sonarRound(&robot,20000,20000,100,&map);

  }
  
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;
}