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
#define gridsize 270
#define PI 3.14159265

int initialize=0;
int finaly, finalx;
int initx, inity;
double angle;

//grid é a estrutura que usamos para marcar cada quadrado do mapa
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

//inicialização do mapa
void newMap(int fx, int fy){
  ////printf("começou a rodar o mapa\n");
  for(int i = 0; i < mapay; i++){
    for(int j = 0; j < mapax; j++){
      pos[i][j] = grid();
    }

  }
  pos[(int) floor(fy/gridsize)][(int) floor(fx/gridsize)].heur = 0;
  //printf("terminou \n");
}

//mapeando atráves do sonar
void sonarRound(ArRobot *thisRobot){
double th=thisRobot->getTh();
int numSonar=thisRobot->getNumSonar(); //Get number of sonar
ArSensorReading* sonarRead;
for (int i = 0; i < numSonar; i++){
  sonarRead = thisRobot->getSonarReading(i);
  ////printf("Sonar %d %f %f\n", i, sonarRead->getX(), sonarRead->getY());
  if(sonarRead->getRange()<5000){

    //marcando floor
    int gridx=(int) floor((sonarRead->getX()/gridsize));
    int gridy=(int) floor((sonarRead->getY()/gridsize));
    if(gridx>0&&gridy>0){
      ////printf("Mapa %d %d %d\n", i, gridx, gridy);
      pos[gridy][gridx].rep='#';
      pos[gridy][gridx].heur=INT_MAX;
    }

    //marcando ceil
    gridx=(int) ceil((sonarRead->getX()/gridsize));
    gridy=(int) ceil((sonarRead->getY()/gridsize));
    if(gridx>0&&gridy>0){
      ////printf("Mapa %d %d %d\n", i, gridx, gridy);
      pos[gridy][gridx].rep='#';
      pos[gridy][gridx].heur=INT_MAX;
    }
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

    bool operator < (const ponto &pontoComparador) const{
      return this->heuristica >= pontoComparador.heuristica;
    }
} ponto;



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
  return ArMath::distanceBetween(array[1],array[0],array1[1],array1[0]);
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
  //printf("You may press escape to exit\n");

  // Collision avoidance actions at higher priority
  ArActionLimiterForwards limiterAction("speed limiter near", 300, 600, 250);
  ArActionLimiterForwards limiterFarAction("speed limiter far", 300, 1100, 400);
  ArActionLimiterTableSensor tableLimiterAction;
  robot.addAction(&tableLimiterAction, 100);
  robot.addAction(&limiterAction, 95);
  robot.addAction(&limiterFarAction, 90);


    FILE *arq;
  arq = fopen("saida.txt","wt");
  FILE * pFile;
  pFile = fopen ("entrada.txt","rt");

  fscanf(pFile,"%d %d %lf\n%d %d", &initx, &inity, &angle,&finalx, &finaly); //posição inicial do robô
  printf("%d %d %lf\n%d %d", initx, inity, angle,finalx, finaly); //
  
  //inicialização forçada 
  /*initx=1000;
  inity=1500;
  angle=0;
  finalx=13000;
  finaly=1300;*/

  newMap(finalx, finaly);

  // Goto action at lower priority
  ArActionGotoStraight gotoPoseAction("goto");
  robot.addAction(&gotoPoseAction, 100);
  
  // Stop action at lower priority, so the robot stops if it has no goal
  ArActionStop stopAction("stop");
  robot.addAction(&stopAction, 80);


  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.enableSonar();
  robot.comInt(ArCommands::SOUNDTOG, 0);

  bool first = true;
  int goalNum = 0;
  ArTime start;
  start.setToNow();

  //marcar a posição inicial do robô no mapa
  robot.moveTo(ArPose(initx,inity,angle),true);//creio que o erro esteja aqui!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //inicializado variaveis atuais
  double coordAtual[2];//[0] == x e [1] == y inicialmente igual aos valores iniciais
  coordAtual[0]=inity;
  coordAtual[1]=initx;
  int gridAtX= (int) floor(robot.getX()/gridsize);  //grid X atual do robô
  int gridAtY= (int) floor(robot.getY()/gridsize);  //grid Y atual do robô

  //coordenadas finais
  double coordFinal[2];
  coordFinal[1] = finalx;
  coordFinal[0] = finaly;
  int gFX= (int) floor(finalx/gridsize); //grid final x
  int gFY=(int) floor(finaly/gridsize);  //grid final y

  //coordenadas temporárias usadas na navegação
  double coordTemp[2];
  ponto atual = ponto( distRet(coordFinal, coordAtual) , coordAtual);

  //estruturas usadas para a escolha dos pontos
  stack< priority_queue<ponto> > pilha;
  priority_queue<ponto>pq;
  stack< ponto > pilhaPonto;
  pilhaPonto.push(atual);

  while (Aria::getRunning()) {
    robot.lock();
    sonarRound(&robot);
    //if(first||gotoPoseAction.haveAchievedGoal()){
      first=false;

      double coordAtual[2];//[0] == x e [1] == y inicialmente igual aos valores iniciais
      coordAtual[0]=robot.getY();
      coordAtual[1]=robot.getX();

      if(pos[gridAtY][gridAtX+1].rep == ' '){
        coordTemp[0] = coordAtual[0];//andar para direita
        coordTemp[1] = coordAtual[1] + gridsize;
        //printf("direita,%f,%f\n",coordTemp[1], coordTemp[0]);
        ////printf("as a%f %f   \n\n", coordTemp[1], coordTemp[0]);
        pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
      }
      if(pos[gridAtY - 1][gridAtX-1].rep == ' '){
        coordTemp[0] = coordAtual[0] - gridsize;//andar para esquerda
        coordTemp[1] = coordAtual[1] - gridsize;
        //printf("sudoeste,%f,%f\n",coordTemp[1], coordTemp[0]);
        ////printf(" asa%f %f   \n\n", coordTemp[1], coordTemp[0]);
        pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
      } if(pos[gridAtY + 1][gridAtX+1].rep == ' '){
        coordTemp[0] = coordAtual[0] + gridsize;//andar para direita
        coordTemp[1] = coordAtual[1] + gridsize;
        //printf("nordeste,%f,%f\n",coordTemp[1], coordTemp[0]);
        ////printf("as a%f %f   \n\n", coordTemp[1], coordTemp[0]);
        pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
      } if(pos[gridAtY+1][gridAtX - 1].rep == ' '){
        coordTemp[0] = coordAtual[0] + gridsize;//andar para cima
        coordTemp[1] = coordAtual[1] - gridsize;
        //printf("noroeste,%f,%f\n",coordTemp[1], coordTemp[0]);
        ////printf(" sasa%f %f   \n\n", coordTemp[1], coordTemp[0]);
        pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
      } if(pos[gridAtY-1][gridAtX + 1].rep == ' '){
        
        coordTemp[0] = coordAtual[0] - gridsize;//andar para baixo
        coordTemp[1] = coordAtual[1] + gridsize;
        //printf("sudeste,%f,%f\n",coordTemp[1], coordTemp[0]);
        ////printf("a sas%f %f   \n\n", coordTemp[1], coordTemp[0]);
        pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
      }
      //Analise dos pontos ao redor

       if(pos[gridAtY][gridAtX-1].rep == ' '){
        coordTemp[0] = coordAtual[0];//andar para esquerda
        coordTemp[1] = coordAtual[1] - gridsize;
        //printf("esquerda,%f,%f\n",coordTemp[1], coordTemp[0]);
        ////printf(" asa%f %f   \n\n", coordTemp[1], coordTemp[0]);
        pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
      }  if(pos[gridAtY+1][gridAtX].rep == ' '){
        coordTemp[0] = coordAtual[0] + gridsize;//andar para cima
        coordTemp[1] = coordAtual[1];
        //printf("cima,%f,%f\n",coordTemp[1], coordTemp[0]);
        ////printf(" sasa%f %f   \n\n", coordTemp[1], coordTemp[0]);
        pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
      } if(pos[gridAtY-1][gridAtX].rep == ' '){
        
        coordTemp[0] = coordAtual[0] - gridsize;//andar para baixo
        coordTemp[1] = coordAtual[1];
        //printf("baixo,%f,%f\n",coordTemp[1], coordTemp[0]);
        ////printf("a sas%f %f   \n\n", coordTemp[1], coordTemp[0]);
        pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
      }

      //inicia a procura pelo melhor ponto para se mover
      ponto topo = pq.top();
      //printf(" primeiro topo %f %f\n",topo.coordenadas[1],topo.coordenadas[0]); 
      //printf("topo heur: %f \n", topo.heuristica);
      //printf("Pilha top heur: %f\n", pilhaPonto.top().heuristica);
      if(topo.heuristica > atual.heuristica){//ponto pior
        //printf("Piorou\n");
        pos[gridAtY][gridAtX].rep = '*';//ponto atual nao é um ponto bom
      for(int i = 0; i < 8; i++){
        if(equalsArrayBi(pilhaPonto.top().coordenadas, topo.coordenadas)){
          //printf("%d if\n", i);
          //printf("IF topo heur: %f \n", topo.heuristica);
          //printf("IF Pilha top heur: %f\n", pilhaPonto.top().heuristica);
          pos[(int) floor(topo.coordenadas[0]/gridsize)][(int) floor(topo.coordenadas[1]/gridsize)].rep = '*';
        }
        pq.pop();
        topo = pq.top();
      }
      pq = pilha.top();
      atual = pilhaPonto.top();
      //printf(" if %f %f\n",atual.coordenadas[1],atual.coordenadas[0]);
      gridAtY=(int) floor(robot.getY()/gridsize);
      gridAtX=(int) floor(robot.getX()/gridsize);
      pilhaPonto.pop();
      pilha.pop();
    } else{
      pilhaPonto.push(atual);//o ponto anterior será o que era atual
      atual = topo;//o atual sera o melhor
      //printf(" else %f %f\n",atual.coordenadas[1],atual.coordenadas[0]);
      pq.pop();
      pilha.push(pq);//e essa pq vai para pilha
      pq = priority_queue<ponto>();
    }
      //atualiza o grid atual
      coordAtual[0] = atual.coordenadas[0];
      coordAtual[1] = atual.coordenadas[1];
      int gridAtX= (int) floor(robot.getX()/gridsize);  //grid X atual do robô
      int gridAtY= (int) floor(robot.getY()/gridsize);  //grid Y atual do robô
      //printf(" escolhido %f %f\n",atual.coordenadas[1],atual.coordenadas[0]);
      
      //ir para o posição nova
      ArPose fut= ArPose(atual.coordenadas[1],atual.coordenadas[0]);
      gotoPoseAction.setGoal(fut);
      
      //impressao do mapa
      for(int i=0; i<100;i++){
         for(int j=0; j<100;j++){
            fprintf(arq,"%c ", pos[i][j].rep);
      } fprintf(arq,"\n");
      }
        fprintf(arq,"\n\n\n");
      robot.unlock();
      ArUtil::sleep(100);
    }
  Aria::exit(0);
  return 0;
}
