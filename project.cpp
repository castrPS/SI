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
  printf("Começou a escolha de passo\n");
  double coordAtual[2];//[0] == x e [1] == y inicialmente igual aos valores iniciais
  coordAtual[0]=inity;
  coordAtual[1]=initx;
  ArPose at= ArPose(coordAtual[1],coordAtual[0],angle);
  double coordFinal[2];
  coordFinal[1] = finalx;
  coordFinal[0] = finaly;
  double coordTemp[2];
  ponto atual = ponto( distRet(coordFinal, coordAtual) , coordAtual);

  stack< priority_queue<ponto, vector<ponto>, compare > > pilha;
  priority_queue<ponto, vector<ponto>, compare >pq;
  stack< ponto > pilhaPonto;
  pilhaPonto.push(atual);
  //while(!equalsArrayBi(coordAtual, coordFinal)){//testa se o ponto atual é igual ao final
    for(int yu = 0; yu < 2; yu++){
    //sonar
    //robot.lock();
    //sonarRound(&robot,coordAtual[1],coordAtual[0],angle);
    //printf("rodou sonar");
    //robot.unlock();

    //Fase de exploracao
    printf("comecou a explorar \n");

    printf("antes:   %f , %f\n", coordAtual[1], coordAtual[0]);
    robot.lock();
    int gridAtY=(int) floor((coordAtual[0])/gridsize);
    int gridAtX=(int) floor((coordAtual[1])/gridsize);
    if(pos[gridAtY-1][gridAtX].rep == ' '){
      coordTemp[0] = coordAtual[0] - 510;//andar para baixo
      coordTemp[1] = coordAtual[1];
      //printf("a sas%f %f   \n\n", coordTemp[1], coordTemp[0]);
      pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
    } if(pos[gridAtY+1][gridAtX].rep == ' '){
      coordTemp[0] = coordAtual[0] + 510;//andar para cima
      coordTemp[1] = coordAtual[1];
      //printf(" sasa%f %f   \n\n", coordTemp[1], coordTemp[0]);
      pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
    } if(pos[gridAtY][gridAtX+1].rep == ' '){
      coordTemp[0] = coordAtual[0];//andar para direita
      coordTemp[1] = coordAtual[1] + 510;
      //printf("as a%f %f   \n\n", coordTemp[1], coordTemp[0]);
      pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
    } if(pos[gridAtY][gridAtX-1].rep == ' '){
      coordTemp[0] = coordAtual[0];//andar para esquerda
      coordTemp[1] = coordAtual[1] - 510;
      //printf(" asa%f %f   \n\n", coordTemp[1], coordTemp[0]);
      pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
    }
    robot.unlock();
    printf("explorou \n");
    //fase de comparacao (ver se o topo da heap e melhor)
    robot.lock();
    ponto topo = pq.top(); 
  printf("heuristica atual: %f \n", atual.heuristica);
  printf("topo heur: %f \n", topo.heuristica);
  printf("Pilha top heur: %f\n", pilhaPonto.top().heuristica);
    if(topo.heuristica > atual.heuristica){//ponto pior
      printf("Piorou\n");
      pos[gridAtY][gridAtX].rep = '*';//ponto atual nao é um ponto bom
      for(int i = 0; i < 4; i++){
        if(!equalsArrayBi(pilhaPonto.top().coordenadas, topo.coordenadas)){
          printf("%d if\n", i);
          printf("IF topo heur: %f \n", topo.heuristica);
  printf("IF Pilha top heur: %f\n", pilhaPonto.top().heuristica);
          pos[(int) floor(topo.coordenadas[0]/gridsize)][(int) floor(topo.coordenadas[1]/gridsize)].rep = '*';
        }
        pq.pop();
        topo = pq.top();
      }
      pq = pilha.top();
      atual = pilhaPonto.top();

      gridAtY=(int) floor((coordAtual[0])/gridsize);
      gridAtX=(int) floor((coordAtual[1])/gridsize);
      
      pilhaPonto.pop();
      pilha.pop();
    } else{

      pilhaPonto.push(atual);//o ponto anterior será o que era atual
      atual = topo;//o atual sera o melhor
      pilha.push(pq);//e essa pq vai para pilha
      pq = priority_queue<ponto, vector<ponto>, compare >();
    }
    coordAtual[0] = atual.coordenadas[0];
    coordAtual[1] = atual.coordenadas[1];
    printf("%f , %f\n", coordAtual[1], coordAtual[0]);
    ArPose fut= ArPose(atual.coordenadas[1]-pilhaPonto.top().coordenadas[1],atual.coordenadas[0]-pilhaPonto.top().coordenadas[0]);
    angle=at.findAngleTo(fut);
    fut= ArPose(atual.coordenadas[1],atual.coordenadas[0]);
    gotoPoseAction.setGoal(fut);
    robot.unlock();
    }
  }
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;
}