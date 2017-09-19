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
#define gridsize 170
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
  //printf("começou a rodar o mapa\n");
  for(int i = 0; i < mapay; i++){
    for(int j = 0; j < mapax; j++){
      pos[i][j] = grid();
    }

  }
  //printf("terminou \n");
  pos[(int) floor(fy/gridsize)][(int) floor(fx/gridsize)].heur = 0;
  //printf("terminou \n");
}

//maeando atráves do sonar
void sonarRound(ArRobot *thisRobot){
double x= thisRobot->getX();
double y= thisRobot->getY();
double th=thisRobot->getTh();
int numSonar=thisRobot->getNumSonar(); //Get number of sonar
ArSensorReading* sonarRead;
//printf("pegou leituras do sonar\n");
//To hold each reading
for (int i = 0; i < numSonar; i++){
  sonarRead = thisRobot->getSonarReading(i);
  //printf("Sonar %d %f %f\n", i, sonarRead->getX(), sonarRead->getY());
  if(sonarRead->getRange()<5000){
    int gridx=(int) floor((sonarRead->getX()/gridsize));

    int gridy=(int) floor((sonarRead->getY()/gridsize));
    if(gridx>0&&gridy>0){
      //printf("Mapa %d %d %d\n", i, gridx, gridy);
      pos[gridy][gridx].rep='#';
      pos[gridy][gridx].heur=INT_MAX;
    }
    gridx=(int) ceil((sonarRead->getX()/gridsize));
    gridy=(int) ceil((sonarRead->getY()/gridsize));
    if(gridx>0&&gridy>0){
      //printf("Mapa %d %d %d\n", i, gridx, gridy);
      pos[gridy][gridx].rep='#';
      pos[gridy][gridx].heur=INT_MAX;
    }
  } 
  }
}

typedef struct ponto{
    float heuristica;
    float x;
    float y;
 
    ponto(double heuristica, float x, float y){
 
        this->heuristica = heuristica;
        this->x = x;
        this->y = y;
    }
 
    bool operator < (const ponto &pontoComparador) const{
      return this->heuristica >= pontoComparador.heuristica;
    }
} ponto;
 
static bool comparePoints(double x1, double y1, double x2, double y2){
    return (x1 == x2) && (y1 == y2);
}
static double distRet(double x1, double y1, double x2, double y2){
  return ArMath::distanceBetween(x1,y1,x2,y2);
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


  // Connect to the robot, get some init data from it such as type and name,
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
  finalx=2000;
  finaly=13000;
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

  //inicialização
  robot.moveTo(ArPose(initx,inity,angle),true);//creio que o erro esteja aqui!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  double coordFinal[2];
  coordFinal[1] = finalx;
  coordFinal[0] = finaly;
  int gFX= (int) ceil(finalx/gridsize); //grid final x
  int gFY=(int) ceil(finaly/gridsize);  //grid final y

  //inicializado variaveis atuais
  double coordAtual[2];//[0] == x e [1] == y inicialmente igual aos valores iniciais
  coordAtual[0]=inity;
  coordAtual[1]=initx;
  int gridAtX= (int) ceil(robot.getX()/gridsize);  //grid X atual do robô
  int gridAtY= (int) ceil(robot.getY()/gridsize);  //grid Y atual do robô

  double tempY;
  double tempX;
  double coordTemp[2];

  stack< priority_queue<ponto> > pilhaHeaps;//pilha de Min-Heaps, para armazenar as já visitadas
  priority_queue<ponto> minHeap;//min-heap de exploração atual
  stack< ponto > anterior;//pilha de ponto anterior, para saber qual foi o último ponto visitado
  int contadorDeAdd;

  ponto atual = ponto (distRet(initx, inity, finalx, finaly), initx, inity);
  anterior.push(atual);

  while (Aria::getRunning()) {
    //robot.lock();
    sonarRound(&robot);
    if(true/*first||gotoPoseAction.haveAchievedGoal()*/){
      first=false;
      contadorDeAdd = 0;//contador para ajudar na hora de uma possivel remocao de itens da heap
      printf("comparacoes\n");
      int ceilx;
      int ceily;

        tempX = atual.x;
        tempY = atual.y+gridsize;
        ceilx=ceil(tempX/gridsize);
        ceily=ceil(tempY/gridsize);
        if(ceilx>=0&&ceily>=0&&pos[ceily][ceilx].rep == ' '){// cima
            
            printf("okay %f %f\n",tempX,tempY);
            minHeap.push(ponto(distRet(tempX, tempY, atual.x, atual.y), tempX, tempY));
            contadorDeAdd++;
        }

        tempX = atual.x+gridsize;
        tempY = atual.y;
        ceilx=ceil(tempX/gridsize);
        ceily=ceil(tempY/gridsize);
        if(ceilx>=0&&ceily>=0&&pos[ceily][ceilx].rep == ' '){// direita
            
            printf("okay %f %f\n",tempX,tempY);
            minHeap.push(ponto(distRet(tempX, tempY, atual.x, atual.y), tempX, tempY));
            contadorDeAdd++;
        } 

        tempX = atual.x+gridsize;
        tempY = atual.y+gridsize;
        ceilx=ceil(tempX/gridsize);
        ceily=ceil(tempY/gridsize);
        if(ceilx>=0&&ceily>=0&&pos[ceily][ceilx].rep == ' '){//Nordeste
            
            printf("okay %f %f\n",tempX,tempY);
            minHeap.push(ponto(distRet(tempX, tempY, atual.x, atual.y), tempX, tempY));
            contadorDeAdd++;
        }

        tempX = atual.x;
        tempY = atual.y-gridsize;
        ceilx=ceil(tempX/gridsize);
        ceily=ceil(tempY/gridsize);
        if(ceilx>=0&&ceily>=0&&pos[ceily][ceilx].rep == ' '){//baixo
            
            printf("okay %f %f\n",tempX,tempY);
            minHeap.push(ponto(distRet(tempX, tempY, atual.x, atual.y), tempX, tempY));
            contadorDeAdd++;
        }

        tempX = atual.x-gridsize;
        tempY = atual.y;
        ceilx=ceil(tempX/gridsize);
        ceily=ceil(tempY/gridsize);
        if(ceilx>=0&&ceily>=0&&pos[ceily][ceilx].rep == ' '){//Esquerda
            
            printf("okay %f %f\n",tempX,tempY);
            minHeap.push(ponto(distRet(tempX, tempY, atual.x, atual.y), tempX, tempY));
            contadorDeAdd++;
        }

        tempX = atual.x-gridsize;
        tempY = atual.y+gridsize;
        ceilx=ceil(tempX/gridsize);
        ceily=ceil(tempY/gridsize);
        if(ceilx>=0&&ceily>=0&&pos[ceily][ceilx].rep == ' '){//Noroeste
            
            printf("okay %f %f\n",tempX,tempY);
            minHeap.push(ponto(distRet(tempX, tempY, atual.x, atual.y), tempX, tempY));
            contadorDeAdd++;
        }

        tempX = atual.x-gridsize;
        tempY = atual.y-gridsize;
        ceilx=ceil(tempX/gridsize);
        ceily=ceil(tempY/gridsize);
        if(ceilx>=0&&ceily>=0&&pos[ceily][ceilx].rep == ' '){//Sudoeste
            
            printf("okay %f %f\n",tempX,tempY);
            minHeap.push(ponto(distRet(tempX, tempY, atual.x, atual.y), tempX, tempY));
            contadorDeAdd++;
        }

        tempX = atual.x+gridsize;
        tempY = atual.y-gridsize;
        ceilx=ceil(tempX/gridsize);
        ceily=ceil(tempY/gridsize);
        if(ceilx>=0&&ceily>=0&&pos[ceily][ceilx].rep == ' '){//Sudeste
            
            printf("okay %f %f\n",tempX,tempY);
            minHeap.push(ponto(distRet(tempX, tempY, atual.x, atual.y), tempX, tempY));
            contadorDeAdd++;
        }


        printf("fim das comparacoes\n");
        //Pontos explorados
        printf("comeco das comparacoes do pontos explorados\n");
        if(minHeap.top().heuristica > atual.heuristica){//nenhum ponto explorado foi melhor
            //Marcar todos os pontos, menos o anterior
            pos[(int) ceil(atual.y/gridsize)][(int) ceil(atual.x/gridsize)].rep = '*';//o ponto atual não será mais visitado, pois ele não levou a algum lugar decente
            for(int cont = 0; cont < contadorDeAdd; cont++){//so percorre a heap o mesmo numero de vezes que algo foi adicionado a ela
                if(comparePoints(minHeap.top().x, minHeap.top().y, anterior.top().x, anterior.top().y)){//evitar marcar o ponto anterior, pois ele será o atual na próxima iteração
                    pos[(int) ceil(minHeap.top().y/gridsize)][(int) ceil(minHeap.top().x/gridsize)].rep = '*';
                }
                minHeap.pop();//tira essse ponto para marcar o próximo
            }
            //com todos os pontos marcados, o ponto atual passa a ser o anterior
            atual = anterior.top();
            anterior.pop();//o anterior agora é o antes do anterior antes
            minHeap = pilhaHeaps.top();
            pilhaHeaps.pop();
        } else {//caso dê no mesmo ou melhorou
            anterior.push(atual);//o ponto anterior é o atual
            atual = minHeap.top();//o ponto atual é o melhor
            pilhaHeaps.push(minHeap);//coloca a atual heap na pilha
            minHeap = priority_queue<ponto>();//esvaziar a atual Minheap
        }
        printf("fim das comparacoes do pontos explorados\n");
        gotoPoseAction.setGoal(ArPose(atual.x,atual.y));
      }
    //robot.unlock();
  }
  // Robot disconnected or time elapsed, shut down
  Aria::exit(0);
  return 0;
}