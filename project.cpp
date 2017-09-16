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
#define gridsize 5100
#define PI 3.14159265
#define gridx (int) floor(mapax/gridsize)
#define gridy (int) floor(mapay/gridsize)
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
void sonarRound(ArRobot *thisRobot, int x, int y, int th)
{

int numSonar;
int i;
//Number of sonar on the robot
//Counter for looping
numSonar = thisRobot->getNumSonar(); //Get number of sonar
printf("pegou numero\n");
ArSensorReading* sonarRead;
printf("pegou leituras do sonar\n");
//To hold each reading
for (i = 0; i < numSonar; i++){
  sonarRead = thisRobot->getSonarReading(i);
  printf("Leitura %d: %f \n", i, sonarRead->getRange());
  //sonarReading->resetSensorPosition(x,y,th);
  //achando o angulo
  double rad= PI / 180.0;
  double angulo= rad*sonarRead->getSensorTh();
  //achando a posição no grid 
  int fx=x+(cos(angulo)*sonarRead->getRange());
  fx=(int) floor(fx/gridsize);
  printf("%d ", fx);
  int fy=y+(sin(angulo)*sonarRead->getRange());
  fy=(int) floor(fy/gridsize);
   printf("%d\n", fy);
  //Só marca como parede se for menor que o limite do sonar
      printf("%c\n", pos[fy][fx].rep);
      pos[fy][fx].rep='#';
      printf("marquei como parede\n");
      pos[fy][fx].heur=INT_MAX;
      pos[fy][fx].x=x+(cos(angulo)*sonarRead->getRange());
      pos[fy][fx].x= y+(sin(angulo)*sonarRead->getRange());
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
      return a.heuristica < b.heuristica;

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
  double x = (array[0] - array1[0])*(array[0] - array1[0]);
  double y = (array[1] - array1[1])*(array[0] - array1[0]);
  return sqrt(x + y);
}

ArPose passo(int x,int y, int th){
  printf("Começou a escolha de passo\n");
  double coordAtual[2];//[0] == x e [1] == y inicialmente igual aos valores iniciais
  coordAtual[0]=y;
  coordAtual[1]=x;
  ArPose at= ArPose(x,y,th);
  double coordFinal[2];
  double coordTemp[2];
  ponto atual = ponto( distRet(coordFinal, coordAtual) , coordAtual);
  ponto anterior = atual;

  stack< priority_queue<ponto, vector<ponto>, compare > > pilha;
  priority_queue<ponto, vector<ponto>, compare >pq;

  while(!equalsArrayBi(coordAtual, coordFinal)){//testa se o ponto atual é igual ao final
    //Fase de exploracao

    if(pos[(int)(int) floor( (coordAtual[0] - 510)/51) ][(int) floor(coordAtual[1]/51)].rep == ' '){
      coordTemp[0] = coordAtual[0] - 510;//andar para baixo
      coordTemp[1] = coordAtual[1];
      pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
    } else if(pos[(int) floor((coordAtual[0] + 510) / 51)][(int) floor(coordAtual[1]/51)].rep == ' '){
      coordTemp[0] = coordAtual[0] + 510;//andar para cima
      coordTemp[1] = coordAtual[1];
      pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
    } else if (pos[(int) floor(coordAtual[0]/51)][(int) floor((coordAtual[1] + 510)/51)].rep == ' ')
    {
      coordTemp[0] = coordAtual[0];//andar para esquerda
      coordTemp[1] = coordAtual[1] + 510;
      pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
    } else if (pos[(int) floor(coordAtual[0]/51)][(int) floor((coordAtual[1] - 510)/51)].rep == ' ')
    {
      coordTemp[0] = coordAtual[0];//andar para direita
      coordTemp[1] = coordAtual[1] - 510;
      pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
    }

    //fase de comparacao (ver se o topo da heap e melhor)
    ponto topo = pq.top(); 
    if(topo.heuristica > atual.heuristica){//ponto pior
      pos[(int) floor(atual.coordenadas[0]/51)][(int) floor(atual.coordenadas[1]/51)].rep = '*';//ponto atual nao é um ponto bom
      for(int i = 0; i < 4; i++){
        if(!equalsArrayBi(anterior.coordenadas, topo.coordenadas)){
          pos[(int) floor(topo.coordenadas[0]/51)][(int) floor(topo.coordenadas[1]/51)].rep = '*';
        }
        pq.pop();
        topo = pq.top();
      }
      pq = pilha.top();
      atual = anterior;
      pilha.pop();
    } else{
      anterior = atual;//o ponto anterior será o que era atual
      atual = topo;//o atual sera o melhor
      pilha.push(pq);//e essa pq vai para pilha
      pq = priority_queue<ponto, vector<ponto>, compare >();
    }
  }
  ArPose fut= ArPose(atual.coordenadas[1],atual.coordenadas[0],0);
  fut= ArPose(atual.coordenadas[1],atual.coordenadas[0],at.findAngleTo(fut));
  return fut;
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

  while (Aria::getRunning()) {

    robot.lock();
    //scanf("%d %d %lf", &initx, &inity, &angle); //posição inicial do robô
  //scanf("%d %d", &finalx, &finaly); //posição final
  initx=1000;
  inity=1500;
  angle=0;
  finalx=13000;
  finaly=13000;
  newMap(finalx, finaly);
  robot.unlock();
  while(true){
    robot.lock();
    sonarRound(&robot,initx,inity,angle);
    printf("rodou sonar");
    robot.unlock();
    robot.lock();
    ArPose ir= passo(initx,inity,angle);
    printf("setou para onde ir");
    robot.unlock();
    robot.lock();
    gotoPoseAction.setGoal(ir);
    robot.unlock();
    robot.lock();
    initx=ir.getX();
    inity=ir.getY();
    angle=ir.getTh();
    robot.unlock();
  }
  }
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;
}