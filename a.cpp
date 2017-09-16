#include <vector>;
#include <math.h>
#include <limits.h>
using namespace std;

class grid {
  public:
    char rep=' '; //representação em char do mapa
    int heur=-1;//valor da heurisitca usado;
    int x;
    int y;
};

class map{
  public:
    vector< vector<grid> > pos;
};

void getSonar(int x, int y, map mapa)
{
for (int i = 0; i < 10; i++)
//Loop through sonar
{
//Get each sonar reading
  double rad= x/ 180.0;
  double angulo= rad*x;
  int fx=floor(x+(cos(angulo)));
  int fy=floor(y+(sin(angulo)));
  if(x<5000){
    //se couber no mapa
    if(mapa.pos.size()>fy && mapa.pos[0].size()>fx){
      mapa.pos[fy][fx].rep='#';
      mapa.pos[fy][fx].heur=INT_MAX;
    }else{
      if(mapa.pos.size()<=fy){
        mapa.pos.resize(fy*2+1);
      }
      if(mapa.pos[0].size()<=fx){
        for(int j=0; j<mapa.pos.size(); j++){
          mapa.pos[i].resize(fx*2+1);
        }
      }
   }
  }

}
}


int main(){
	vector< vector<grid> > mapa();
}