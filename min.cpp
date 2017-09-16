#include <bits/stdc++.h>
using namespace std;

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
bool equalsArrayBi (double array[2], double array1[2]){
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

int main(){
	mapa[][];
	double coordAtual[2];//[0] == x e [1] == y inicialmente igual aos valores iniciais
	double coordFinal[2];
	double coordTemp[2];
	ponto atual = ponto( distRet(coordFinal, coordAtual) , coordAtual);
	while(!compareArrayBi(coordAtual, coordFinal)){//testa se o ponto atual é igual ao final
		//Fase de exploracao
		priority_queue<Node, vector<Node>, compare >pq;
		if(mapa[coordAtual[0] - 51][coordAtual[1]] != '#'){
			coordTemp[0] = coordAtual[0] - 51;//andar para a esquerda
			coordTemp[1] = coordAtual[1];
			pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
		} else if(mapa[coordAtual[0] + 51][coordAtual[1]] != '#'){
			coordTemp[0] = coordAtual[0] + 51;//andar para a direita
			coordTemp[1] = coordAtual[1];
			pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
		} else if (mapa[coordAtual[0]][coordAtual[1] + 51] != '#')
		{
			coordTemp[0] = coordAtual[0];//andar para cima
			coordTemp[1] = coordAtual[1] + 51;
			pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
		} else if (mapa[coordAtual[0]][coordAtual[1] - 51] != '#')
		{
			coordTemp[0] = coordAtual[0];//andar para baixo
			coordTemp[1] = coordAtual[1] - 51;
			pq.push(ponto( distRet(coordTemp, coordFinal) , coordTemp));
		}

		//fase de comparacao (ver se o topo da heap e melhor)
		ponto topo = pq.top();
		pq.pop();//tirar ponto atual da pq, pois caso ele seja ruim explora o "segundo ponto melhor na proxima" 
		if(topo.heuristica > atual.heuristica){//ponto pior
			for (int i = 0; i < 4; ++i)
			{
				topoTemp = pq.top();
				mapa[topoTemp.coordenadas[0]][topoTemp.coordenadas[1]] = '#';
				pq.pop();
			}
		}

	}
}