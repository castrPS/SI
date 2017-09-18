#include <vector>
#include <math.h>
#include <limits.h>
#include "Aria.h"
#include <fstream>
#include <bits/stdc++.h>

using namespace std;

#define barreira INT_MAX
#define mapax 1000
#define mapay 1000
#define gridsize 510
#define PI 3.14159265

int initialize=0;
int finaly, finalx;
int initx, inity;
double angle;


typedef struct grid{
    char rep; //representação em char do mapa
    int heur;//valor da heurisitca usado;
    int x;
    int y;

    grid(){

        this->rep=' ';
        this->heur=0;
    }

} grid;


//mapa
grid pos[1000][1000];


//leitura de arquivo e inicialização do mapa e posição do robô
void newMap(int fx, int fy){
  printf("começou a rodar o mapa\n");
  for(int i = 0; i < mapay; i++){
    for(int j = 0; j < mapax; j++){
      pos[i][j] = grid();
    }

  }
  printf("terminou \n");
  pos[(int) floor(fy/gridsize)][(int) floor(fx/gridsize)].heur = 0;
  printf("terminou \n");
}

//maeando atráves do sonar
void sonarRound(ArRobot *thisRobot){
double x= thisRobot->getX();
double y= thisRobot->getY();
double th=thisRobot->getTh();
int numSonar=thisRobot->getNumSonar(); //Get number of sonar
ArSensorReading* sonarRead;
printf("pegou leituras do sonar\n");
//To hold each reading
for (int i = 0; i < numSonar; i++){
  sonarRead = thisRobot->getSonarReading(i);
  printf("Sonar %d %f %f\n", i, sonarRead->getX(), sonarRead->getY());
  if(sonarRead->getRange()<5000){
  int gridx=(int) floor((sonarRead->getX()/gridsize));
  int gridy=(int) floor((sonarRead->getY()/gridsize));
  printf("Mapa %d %d %d\n", i, gridx, gridy);
  pos[gridy][gridx].rep='#';
  pos[gridy][gridx].heur=INT_MAX;
  } 
  }
}

typedef struct ponto{
    double heuristica;
    double *coordenadas;

    ponto(double heuristica, double coordenadas[2]){

        this->heuristica = heuristica;
        this->coordenadas = new double[2];
        this->coordenadas = coordenadas;
    }

} ponto;


struct compare{
  bool operator()(const ponto& a, const ponto& b){
      printf("%f   %f  \n\n\n", a.heuristica, b.heuristica);
      return a.heuristica > b.heuristica;

    }
};
static bool equalsArrayBi (double array[2], double array1[2]){
  bool final = true;
  for (int i = 0; i < 2; ++i)
  {
    if(array[i] != array1[i])
      final = false;
  }
  return final;
}
static double distRet(double array[2], double array1[2]){
  double xa = (array[0] - array1[0])*(array[0] - array1[0]);
  double ya = (array[1] - array1[1])*(array[1] - array1[1]);
  return sqrt(xa + ya);
}

static void passo(ArRobot *robot, int x,int y, int th){
  
  
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


  //scanf("%d %d %lf", &initx, &inity, &angle); //posição inicial do robô
  //scanf("%d %d", &finalx, &finaly); //posição final
  initx=1000;
  inity=1500;
  angle=0;
  finalx=13000;
  finaly=13000;
  newMap(finalx, finaly);

  // Goto action at lower priority
  ArActionGoto gotoPoseAction("goto");
  robot.addAction(&gotoPoseAction, 50);
  
  // Stop action at lower priority, so the robot stops if it has no goal
  ArActionStop stopAction("stop");
  robot.addAction(&stopAction, 40);


  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.enableSonar();
  robot.comInt(ArCommands::SOUNDTOG, 0);

  bool first = true;
  int goalNum = 0;
  ArTime start;
  start.setToNow();



  //inicialização
  robot.moveTo(ArPose(initx,inity,angle),true);
  

  while (Aria::getRunning()) {
    sonarRound(&robot);
    
  }
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;
}