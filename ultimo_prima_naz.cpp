#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#define dst 20
#include <stdio.h>
#include<cstdlib>
#include<vector>
#include<queue>
#include<map>
#include<math.h>
#include<fstream>

//#define ii pair<int,int>
#define coppia pair<int,int>

using namespace cv;
using namespace std;

//Righe aggiunte da "programma.cpp"
int fd;
int id_message;

#include "seriale.h"
#include "motors.h"
//#include "servo.h"

///---------------------funzioni  e variabili di utils

Mat frame;
Mat edges;
Mat thr;
Mat drawing;
Mat lns=Mat::zeros( 160, 120, CV_8UC3 );

int erosion_elem = 0;
int erosion_size = 2;
int dilation_elem = 0;
int dilation_size = 2;
int const max_elem = 2;
int const max_kernel_size = 21;

void Erosion( int, void*, Mat& imm ){
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  erode( imm, imm, element );
}

/** @function Dilation */
void Dilation( int, void*, Mat& immagine ){
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( immagine, immagine, element );
}

void MyLine( Mat img, Point start, Point end ){
  int thickness = 2;
  int lineType = 8;
  line( img,
    start,
    end,
    Scalar( 0, 0, 255 ),
    thickness,
    lineType );
}

void MyFilledCircle( Mat img, Point center ){
 int thickness = -1;
 int lineType = 8;

 circle( img,
         center,
         3,
         Scalar( 0, 255, 0 ),
         thickness,
         lineType );
}

void MyCenterCircle( Mat img, Point center ){
 int thickness = -1;
 int lineType = 8;

 circle( img,
         center,
         5,
         Scalar( 255, 0, 0 ),
         thickness,
         lineType );
}

void DetectLine(int level){
        //MyLine(frame,Point(20,460-dst*level),Point(620,460-dst*level));
        int a=0;
        int lastcoord;
        uchar intensity=thr.at<uchar>(460-dst*level,20);
        bool isline=false;
        if(intensity==0)
        {
            MyFilledCircle(frame,Point(20,460-dst*level));
            isline=true;
            lastcoord = 20;
            //a=50;
        }
        for(int i=21+a;i<=620;i++)
        {
            //cout << i<<endl;
            intensity=thr.at<uchar>(460-dst*level,i);
            if(isline==true&&(intensity==255||i==620))
            {
                isline=false;
                MyFilledCircle(frame,Point(i,460-dst*level));
                MyCenterCircle(frame,Point((i+lastcoord)/2,460-dst*level));
                //i+=100;
            }
            else if(isline==false&&intensity==0)
            {
                isline=true;
                MyFilledCircle(frame,Point(i,460-dst*level));
                lastcoord=i;
            }
        }

  //return pos;
}

///-------------------------------funzioni e variabili del programma

struct Mypoint{
int x;
int y;
int dist;

};


coppia iimake(int a, int b)
{
    return (make_pair(a, b));
}

int colorazione[3]={0,0,0};


void calcola_colore(int dist){

int qualecolore = (dist/250)%3;
int intensita=(dist%250);

for(int i=0; i<3; i++)
    colorazione[i]=0;

colorazione[qualecolore]=intensita;

}

///VARIABILI DEL "ROBOT"-------------------
int soglianero=120;
int fino_a_dove=-1;   ///la distanza cui arriva LA LINEA PRINCIPALE
int distanza_isole=-1; ///vedi sotto
int quanti_per_isola=0; ///QUANTI PER ISOLA: quanto è la distanza MASSIMA tra un pixel a una data distanza (distanza isole) di modo che si possano considerare
                        ///NON separati, cioè diciamo riunibili in uno stesso punto. Secondo la geometria, dovrebbe essere addirittura 1, non di più.

///VARIABILI PER LA FUNZIONE--------------
map<coppia, int> distanze; ///la matrice dove il pixel in pos(x,y) è a distanza d dal primo nero trovato
int distanza_max=-1;

vector<int>vettore_x[10000];
vector<int>vettore_y[10000];///10mila vuol dire che possiamo arrivare fino alla distanza di 10mila pixel (esagerato) (vedi NOTA)

vector<int> hdng_isole;
vector<int> hd_copia;
int linea_x[10000];     ///i due vettori che si salvano la media delle coord dei punti della linea PRINCIPALE a distanza [i]
int linea_y[10000];
int last_errore=0;

queue<Point> coda;
const float angolo_visione_cam = 1.05;

Mat colore, grigio, copiacolore;

#define DEBUG 0
#define DEBUG2 10 ///è per le cose importanti, che aiutano a leggere

bool compreso(int x, int a, int b){
    if(a>b) swap(a,b);
    if(a <= x && x<=b) return true;
    return false;
}

int angolo_tra(float x1, float y1, float x2, float y2){ ///RITORNA L'ANGOLO 0-360 TRA DUE PUNTI
    swap(y1,y2);
    if(x1==x2)
        if(y1<y2) return 90;
        else      return 270;
    if(y1==y2)
        if(x1<x2) return 0;
        else      return 180;


    int angolo=(atan((y2-y1)/(x2-x1))*57.3);
    //cout<<"                 angolo tra due punti"<<angolo<<endl;
    /**short seno;
    if(y2 > y1) seno=+1;
    else        seno=-1;

    f(angolo * seno > 0) return angolo;
    else                  return angolo+180; fuck this shit! non worka un cazzo*/

    ///PRIMO QUADRANTE:
    if(x2 > x1 && y2 > y1) return angolo;
    ///SECONDO QUADRANTE:
    if(x2 < x1 && y2 > y1) return angolo + 180;
    ///TERZO QUADRANTE:
    if(x2 < x1 && y2 < y1) return angolo +180;
    ///QUARTO QUADRANTE:
    if(x2 > x1 && y2 < y1) return angolo+360;
}

int angolo_medio_linea(int lineax[], int lineay[], int lungh, int l, int step){
	long long int media=0;
	int t=0, c=0;
	for(int i=0; i<(lungh-l); i+=step){
		c++;

			t=angolo_tra(lineax[i],lineay[i],lineax[i+l],lineay[i+l]);
			media+=t;
		//cout<<"TEMP ANGOLO:"<<t<<endl;
		#if DEBUG2

        colore.at<Vec3b>(lineay[i+l],lineax[i+l]) = (Vec3b){0,0,255};
		colore.at<Vec3b>(lineay[i],lineax[i]) = (Vec3b){255,255,255};
		#endif // DEBUG
	}
	media=media/c;
        #if DEBUG
        //cout<<media<<endl;
        //cout<<angolo_tra(lineax[0],lineay[0],lineax[lungh],lineay[lungh]);
        #endif // DEBUG
	return media;
}

coppia trova_primo_nero(int base /**,int metodo=?*/){
    /**
    !!IMPORTANTE: SE SI TIENE IL FOR DELLE I ALLORA SI FA CONTINUO, SE NON SI TIENE VIENE FUORI A SALTI!!!
    */

    int primox, primoy, lastx;
    bool trovato=false;
    ///NOTA PER TUTTO STO CESSO
    ///IL MAT::AT(A,B) FUNZIONA CHE A SONO LE RIGHE (QUINDI LA Y) E B LE COLONNE (LA X)
    #if DEBUG
    assert(base >=0);
    #else
    return make_pair(-1,-1);
    #endif // DEBUG
    int i = base;
    //for(int i=base; i>=0; i--)  ///i:righe
    //{
        for(int j=grigio.cols -1; j>=0; j--) ///j:colonne; CERCA IL PRIMO NERO A PARTIRE DA DESTRA
        {
            if ((int)grigio.at<uchar>(i,j) <=soglianero )
            {
                primox=j;
                primoy=i;
                trovato=true;
                j=-1;///esci dal for
            }
             colore.at<Vec3b>(i,j) = (Vec3b){	0, 102, 255};
        }

        for(int j=0; j<grigio.cols && trovato; j++) ///QUANDO HA TROVATO IL PRIMO NERO A DESTRA, LO CERCA DA SINISTRA
        {
            if ((int)grigio.at<uchar>(i,j) <=soglianero )
            {
                lastx=j;
                j=grigio.cols+10;
                i=-1;  ///QUANDO HA TROVATO IL NERO DX E IL NERO SX sulla linea di base,
                        ///esce da tutti i for possibili, ha trovato le cordinate esatte del primo nero
            }
        }
    primox=(primox+lastx)/2;
    //}
    if(!trovato) return make_pair(-1,-1);
    return make_pair(primox,primoy);
}

bool bfs_principaple(Mat& grigio, bool inizio, int primox=-2, int primoy=-2){ ///COLORA "colore" e riempie la matrice DISTANZE
    ///NOTA PER TUTTO STO CESSO
    ///IL MAT::AT(A,B) FUNZIONA CHE A SONO LE RIGHE (QUINDI LA Y) E B LE COLONNE (LA X)
    if(inizio){
    int lastx;
    for(int i=grigio.rows-1; i>=0; i--)  ///i:righe
    {
        for(int j=grigio.cols -1; j>=0; j--) ///j:colonne; CERCA IL PRIMO NERO A PARTIRE DA DESTRA
        {
            if ((int)grigio.at<uchar>(i,j) <=soglianero )
            {
                primox=j;
                primoy=i;
                j=-1;///esci dal for
            }
        }

        for(int j=0; j<grigio.cols; j++) ///QUANDO HA TROVATO IL PRIMO NERO A DESTRA, LO CERCA DA SINISTRA
        {
            if ((int)grigio.at<uchar>(i,j) <=soglianero )
            {
                lastx=j;

                //escimi il for
                j=grigio.cols+10;
                i=-1;  ///QUANDO HA TROVATO IL NERO DX E IL NERO SX CHE STANNO PIÙ IN BASSO DI TUTI,
                        ///esce da tutti i for possibili, ha trovato le cordinate esatte del primo nero
            }
        }

    }
    primox=(primox+lastx)/2;
    }
    if(inizio && primoy < 30) return false;
    #if DEBUG2
    colore.at<Vec3b>(primoy, primox)=(Vec3b){0,0,250};
    //cout<<"HO TROVATO UN NERO!"<<primox<<" "<<primoy<<endl;
    #endif // DEBUG2

    ///--------------------<<BFS>>---------------///

    ///SETPAN
    /*
    cout << "PRIMO_X → " << primox << endl;
    if(primox>grigio.cols*2/3)
        setPan(80);
    else if(primox<grigio.cols/3)
        setPan(100);
    else if(primox < grigio.cols *2/3 -10 && primox > grigio.cols/3 +10);
        setPan(90);
    */
    coda.push(Point(primox, primoy));
    distanze[iimake(primox,primoy)]=0;

    int xcorrente, ycorrente,xprossimo,yprossimo, distcorr;

    while(!coda.empty())
    {
        Point corrente = coda.front();
        coda.pop();
        xcorrente=corrente.x;
        ycorrente=corrente.y;
        distcorr= distanze[iimake(xcorrente,ycorrente)];

        if(distcorr> distanza_max)
            distanza_max=distcorr;
        vettore_x[distcorr].push_back(xcorrente);
        vettore_y[distcorr].push_back(ycorrente);

                    #if DEBUG2
                    //calcola_colore(distcorr);  ///colora il punto sulla immagine
                    //colore.at<Vec3b>(ycorrente,xcorrente) = (Vec3b){colorazione[0],colorazione[1],colorazione[2]};
                    #endif // DEBUG2

        for(int i=-1; i<=1; i++)
            for(int j=-1; j<=1; j++)   ///PER OGNI VICINO DEL PIXEL CORRENTE CONTROLLA SE LO DEVI PRENDERE
            {
                xprossimo = xcorrente+i;
                yprossimo = ycorrente+j;

                if  ((xprossimo>=0 && xprossimo<grigio.cols)
                    &&
                    (yprossimo>=0 && yprossimo<grigio.rows)  ///SE È DENTRO I CONFINI DELLA MATRICE,
                    &&
                    (distanze.count(iimake(xprossimo,yprossimo)) == 0 ) ///BFS:SE NON LO HAI GIA PRESO IN CONSIDERAZIONE,
                    &&
                    ( (int)grigio.at<uchar>(yprossimo,xprossimo) <= soglianero)) ///SE È UN NERO,
                    {
                        coda.push(Point(xprossimo,yprossimo));    ///metti il prossimo nodo nella coda
                        distanze[iimake(xprossimo,yprossimo)] =  distcorr+1; ///setta la distanza del prossimo a la tua +1
                    } //endif
            } //endfor


    }//endwhile; fine bfs
    return true;
}

void tratta_linea_principale(){
    ///--<<CALCOLO DELLA MEDIA, PER VEDERE DOVE VA LA TRAIETTORIA DELLA LINEA>>----
    int mediax;
    int mediay;
    for(int i=0; i<=fino_a_dove; i++)
    {
        mediax=0;
        mediay=0;
        for(int j=0; j<vettore_x[i].size(); j++)
        {
            mediax=mediax+vettore_x[i][j];
            mediay=mediay+vettore_y[i][j];
        }
        ///NOTA: LA DIVISIONE INTERA CI PIACE PERCHÈ MAINLY ESSENDO COORDINATE CE NE SBATTIAMO IL CAZZ!
        #if DEBUG2
            if(vettore_x[i].size()==0 || vettore_y[i].size()==0)
                cout<<"DIVISIONE PER ZEROH!!!!!\n";
        #endif // DEBUG2
        mediax=mediax /vettore_x[i].size();
        //mediax=(int)mediax;
        mediay=mediay /vettore_y[i].size();
        //mediay=mediay;

        linea_x[i]=(int)mediax;
        linea_y[i]=(int)mediay;
        #if DEBUG2
        colore.at<Vec3b>(linea_y[i],linea_x[i])=(Vec3b){250,250,250};
        #endif // DEBUG2
    }
    }

int bfs_isole(Mat& grigio,int distanza_isole_funz){
        ///----LA CONTROBFS PER VEDERE QUANTE sono le isole
    map<pair<int, int>, bool> usati;

    int numero_isole=0;
    for(int i=0; i<vettore_x[distanza_isole_funz].size(); i++)
        if(usati.count(iimake(vettore_x[distanza_isole_funz][i],vettore_y[distanza_isole_funz][i]))==0)
        {
            numero_isole++;
            usati[iimake(vettore_x[distanza_isole_funz][i],vettore_y[distanza_isole_funz][i])]=true;
            coda.push(Point(vettore_x[distanza_isole_funz][i],vettore_y[distanza_isole_funz][i]));

            while(!coda.empty()) ///NOTA, ERA VERAMENTE UN CESSO DA IMPLEMENTARE IL FATTO DELLA DISTANZA MINIMA TRA I PIXEL ISOLA, COSÌ LASCIAI PERDERE
            {

                Point corrente = coda.front();
                coda.pop();
                int xcorrente=corrente.x;
                int ycorrente=corrente.y;

                for(int i=-1; i<=1; i++)
                    for(int j=-1; j<=1; j++)   ///PER OGNI VICINO DEL PIXEL CORRENTE CONTROLLA SE LO DEVI PRENDERE
                    {
                        int xprossimo = xcorrente+i;
                        int yprossimo = ycorrente+j;
                        coppia  proxnodo=iimake(xprossimo,yprossimo);
                        if((xprossimo>=0 && xprossimo<grigio.cols) && (yprossimo>=0 && yprossimo<grigio.rows)  ///SE È DENTRO I CONFINI DELLA MATRICE,
                            &&
                            (usati.count(proxnodo) == 0 )&&  (abs(distanze[proxnodo]-distanza_isole_funz) <=quanti_per_isola) && ///BFS:SE NON LO HAI GIA PRESO IN CONSIDERAZIONE,
                            ( (int)grigio.at<uchar>(yprossimo,xprossimo) <= soglianero) ///SE È UN NERO,
                            )

                        {
                            usati[proxnodo]=true;
                            coda.push(Point(xprossimo,yprossimo));
                            #if DEBUG2
                            colore.at<Vec3b>(yprossimo,xprossimo) = (Vec3b){250,250,0};
                            #endif // DEBUG2

                        }
                    }
            }//endwhile

    }

    return numero_isole;
}//endfunction

void heading_isole(Mat& grigio){  ///PROBLEMA → COSA FA QUESTA FUNZIONE !?! COSA SONO GLI HEADING DELLE ISOLE? (ballerin)
    map<coppia, bool> usati;
    Point corrente;
    int xprossimo,yprossimo;
    coppia proxnodo, nodocorr;
    int mediax[2000], mediay[2000];
    int quantix[2000], quantiy[2000]; //non mettiamo i vector del cazzo, sono lenti

    int dist_proxnodo, dist_corr;

    for(int i=0; i<vettore_x[distanza_isole].size(); i++)
        if(usati.count(iimake(vettore_x[distanza_isole][i],vettore_y[distanza_isole][i])) == 0)
        {

           for(int k=0; k<2000; k++)
            {
                mediax[k]=0;
                mediay[k]=0;
                quantix[k]=0;
                quantiy[k]=0;
            }
            int max_dist_isola=0;

            coda.push(Point(vettore_x[distanza_isole][i],vettore_y[distanza_isole][i]));
                #if DEBUG
                cout<<"             non ho ancora crashato \n";
                #endif // DEBUG
            while(!coda.empty()){
                corrente=coda.front();
                coda.pop();
                nodocorr = iimake(corrente.x,corrente.y);
                dist_corr=distanze[nodocorr];

                mediax[dist_corr-distanza_isole]+=corrente.x;
                mediay[dist_corr-distanza_isole]+=corrente.y;
                quantix[dist_corr-distanza_isole]++;
                quantiy[dist_corr-distanza_isole]++;

                if(dist_corr > max_dist_isola)
                    max_dist_isola = dist_corr;

                for(int k=-1; k<=1; k++)
                    for(int j=-1; j<=1; j++)   ///PER OGNI VICINO DEL PIXEL CORRENTE CONTROLLA SE LO DEVI PRENDERE
                    {
                        xprossimo = corrente.x+k;
                        yprossimo = corrente.y+j;
                        proxnodo=iimake(xprossimo,yprossimo);
                        dist_proxnodo=distanze[proxnodo];


                        if((xprossimo>=0 && xprossimo<grigio.cols) && (yprossimo>=0 && yprossimo<grigio.rows)  ///SE È DENTRO I CONFINI DELLA MATRICE,
                            &&
                            ( (int)grigio.at<uchar>(yprossimo,xprossimo) <= soglianero)&& ///SE È UN NERO,
                            (dist_proxnodo >= distanza_isole) && (usati.count(proxnodo) == 0 )  ///BFS:SE NON LO HAI GIA PRESO IN CONSIDERAZIONE,
                            )

                        {
                            usati[proxnodo]=true;
                            coda.push(Point(xprossimo,yprossimo));

                        }
                    }
            }///endwhile
       // cout<<"NO CRASH 1\n";
        for(int k=0; k<max_dist_isola-distanza_isole && quantix[k]>=1; k++)
        {
            mediax[k]=(int)(mediax[k]/quantix[k]);
            mediay[k]=(int)(mediay[k]/quantiy[k]);
        }
        //cout<<"no crash 2\n";//getchar();
        cout<<"max dist isola "<<max_dist_isola -distanza_isole<<endl;
        if(max_dist_isola -distanza_isole>= 30)
        hdng_isole.push_back(angolo_medio_linea(mediax,mediay,max_dist_isola-5-distanza_isole, max_dist_isola-5-distanza_isole-1,max_dist_isola-5-distanza_isole+4));
        else
        hdng_isole.push_back(-5);

                            #if DEBUG2
                            if(max_dist_isola -distanza_isole< 30)
                            cout<<"ISOLA TROPPO CORTAH!\n";
                            cout<<"angolo isola"<<hdng_isole.size()<<" "<<hdng_isole[hdng_isole.size()-1]<<endl;
                            /**cout<<"SONO IN UN ISOLA: FACCIAMO IL DEBUGGONE\n";
                            cout<<"QUANTI E LA MEDIA A 0 E 1  "<<quantix[0]<<" "<<mediax[0]<<" "<<quantix[1]<<" "<<mediax[1]<<endl;
                            cout<<"direzione dell isola"<<hdng_isole.size()<<" "<<hdng_isole[hdng_isole.size()-1]<<endl;getchar();*/
                            #endif // DEBUG2
        }///endif/for


return ;
}

bool condizione_di_gap(int x, int y){
    if((x < grigio.cols - 200) && x > 200 &&
      y > 150)
        return true;
    else
        return false;
}

void calibraVerde(Mat& imm){
    imwrite("test.png",imm);
    int maxgreen = 0, minr=400, minb=500;

    for(int r =0; r < imm.rows; r++)
        for(int c = 0; c < imm.cols; c++)
        {
            Vec3b intensity = imm.at<Vec3b>(r, c);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];

            maxgreen=min(maxgreen,(int)green);
            minb=max(minb,(int)blue);
            minr=max(minr,(int)red);
        }

    cout<<"        i minimi di g, r, b sono"<<maxgreen<<"\n"<<minb<<"\n"<<minr<<"\n";//getchar();
}

Mat tresholda_verde(Mat& imm, int bval, int gval, int rval){
    /*
    Mat tbin(imm.rows, imm.cols, CV_8UC1,Scalar(0));
    int mingreen = 0, maxr=400, maxb=500;

    for(int r =0; r < imm.rows; r++)
        for(int c = 0; c < imm.cols; c++)
        {
            Vec3b intensity = imm.at<Vec3b>(r, c);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];
            //cout<<(int)green<<endl;
            if(( (int)green > gval) && ((int)blue < bval) && ((int)red < rval)){
                tbin.at<uchar>(r,c)=(uchar)(255);
            }

        }

        return tbin;
        */

  	// Convert input image to HSV
  	Mat hsv_image;
    cvtColor(imm, hsv_image, cv::COLOR_BGR2HSV);

  	// Threshold the HSV image, keep only the greenh pixels
  	Mat gr_range;
  	inRange(hsv_image, cv::Scalar(38, 100, 100), cv::Scalar(75, 255, 255), gr_range);
 	//inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
 	return gr_range;
}

Point trova_verde(){
//calibraVerde(copiacolore);
    #if DEBUG
    //cout<<"         RICERCA FOR VERDI!\n";
    #endif // DEBUG
Mat matVerdi = tresholda_verde(copiacolore, 65, 75,65);
Dilation(0,0,matVerdi);
Dilation(0,0,matVerdi);
Dilation(0,0,matVerdi);
Dilation(0,0,matVerdi);
  //imwrite("biancoverde.png", matVerdi);getchar();
  //imshow("Colore",matVerdi);


SimpleBlobDetector::Params params;

	// Change thresholds
params.minThreshold = 10;
params.maxThreshold = 200;

// Filter by Circularity
params.filterByCircularity = false;
params.filterByArea = false;
params.filterByColor = true;
params.filterByConvexity = false;
params.filterByInertia = false;
params.minDistBetweenBlobs =100;

params.blobColor = 255;
params.maxArea = 100000;
params.minArea = 200;

Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

std::vector<KeyPoint> keypoints;
detector->detect( matVerdi, keypoints);
    #if DEBUG2
    cout<<"HO TROVATO "<<keypoints.size()<<" VERDI\n";
    drawKeypoints( matVerdi, keypoints, copiacolore, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imwrite("col.png",copiacolore);
    #endif // DEBUG2



if(keypoints.size()==0)
    return Point(-1,-1);
return Point(keypoints[0].pt.x,keypoints[0].pt.y);
}

char tratta_verde_linea(Point2i verde, Point2i lin){

    int angolo = angolo_tra(lin.x,lin.y,verde.x,verde.y);
        #if DEBUG
        cout<<"      "<<lin.x<<" "<<lin.y<<" "<<verde.x<<" "<<verde.y<<"\n";
        #endif // DEBUG
        #if DEBUG2
        cout<<"     angolo verde : "<<angolo<<endl;
        #endif // DEBUG2

    ///COME SI MISURANO GLI ANGOLI DEL VERDE? (ballerin)


    if((260>angolo) && (angolo >100))
        return ('s');
    if((angolo < 80)|| (angolo > 280))
        return ('d');

    return('e');


}

float calcola_errore(int hdLinea, int xIn, int yIn=-2, int xFin = -2, int yFin = -2){
    float errore = 0;
    ///PONIAMO CHE SIA NEGATIVO IL ROBOT GIRA A SX, POSITIVO A DX
    float k_angolo = 2.5, k_posLinea=2.0;
        #if DEBUG
        //cout<<"        per la hd : "<<(90-hdLinea)<<endl<<"per la pos : "<<(xIn - (grigio.cols/2))<<endl;
        //cout<<"         x iniziale "<<xIn<<" le colonne sono "<<grigio.cols<<endl;
        #endif // DEBUG
    errore += (90-hdLinea) * k_angolo;
    errore += (xIn - (grigio.cols/2)) * k_posLinea;

    return errore;
}
/**
     _                                          _                       _  _
    | |                                        (_)                     | |(_)
  __| |  ___   _ __    ___    _ __  ___   __ _  _   ___   _ __    __ _ | | _
 / _` | / _ \ | '_ \  / _ \  | '__|/ _ \ / _` || | / _ \ | '_ \  / _` || || |
| (_| || (_) || |_) || (_) | | |  |  __/| (_| || || (_) || | | || (_| || || |
 \__,_| \___/ | .__/  \___/  |_|   \___| \__, ||_| \___/ |_| |_| \__,_||_||_|
              | |                         __/ |
              |_|                        |___/
*/

Point2f trovaPalla(Mat& imm, Ptr<SimpleBlobDetector> palleDetector){

    Mat imm2;
    vector<KeyPoint> palleKeypoints;
    imm = tresholda_verde(imm,100,80,80);
    Dilation(0,0,imm);
    Dilation(0,0,imm);
    Dilation(0,0,imm);
    Dilation(0,0,imm);

    //imshow("cacca",imm);
    //waitKey(0);

    palleDetector->detect( imm, palleKeypoints);
            #if DEBUG2
            cout<<"ne ho trovati "<<palleKeypoints.size()<<endl;
            drawKeypoints( imm, palleKeypoints, imm2, Scalar(0,250,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
            //imshow("cosa_trovato",imm2);
            for(int i=0; i<palleKeypoints.size(); i++){
                cout<<"HO TROVATO  QUALCOSA a: x"<<(int)palleKeypoints[i].pt.x<<" y "<<(int)palleKeypoints[i].pt.y<<endl;
                //waitKey(0);
            }
            #endif // DEBUG2

    if(palleKeypoints.size() >=1)
      return palleKeypoints[0].pt;
    else
      return Point2f(-1,-1);
}


void main_Ultimastanza(VideoCapture& cap){
  Mat imm,imm2;

SimpleBlobDetector::Params params;

    params.minThreshold = 10;
    params.maxThreshold = 200;

    params.filterByCircularity = false;
    params.filterByArea = true;
    params.filterByColor = true;
    params.filterByConvexity = false;
    params.filterByInertia = false;
    params.minDistBetweenBlobs = 30;

    params.minCircularity = 0.3;
    params.maxCircularity = 1.7;
    params.blobColor = 255;
    params.maxArea = 1000;
    params.minArea = 0;

Ptr<SimpleBlobDetector> palleDetector = SimpleBlobDetector::create(params);
cout<<"STO UTILIZZANDO "<<CV_MAJOR_VERSION<<endl;

for(;;){
    for(int contrl=0;contrl<5;contrl++)
        cap >> imm; // get a new frame from camera
        // Setup SimpleBlobDetector parameters.
    //imshow("roba",imm);
    //waitKey(0);
    Point2f palla = trovaPalla(imm,palleDetector);
    if(palla.x != -1){
    int angoloFin=angolo_visione_cam * (palla.x - imm.cols/2)/(imm.cols/2);
        //MUOVI_CAMERA (angoloFin=);
        cout<<"muovo ad angolo :"<<angoloFin<<endl;
    }

}
return;
}


int check_curva_nov(){ ///SERVE A VERIFICARE SE VIENE TROVATA UNA CURVA A NOVANTA
    int curva_nov_dest=1;
    int curva_nov_sinis=1;
    for(int i=100; i<=140; i++){
        if ((int)grigio.at<uchar>(60,i) >soglianero && (int)grigio.at<uchar>(65,i) >soglianero && (int)grigio.at<uchar>(55,i) >soglianero){
            curva_nov_dest=0;
            break;
        }
    }
    for(int i=20; i<=80; i++){
        if ((int)grigio.at<uchar>(60,i) >soglianero && (int)grigio.at<uchar>(65,i) >soglianero && (int)grigio.at<uchar>(55,i) >soglianero){
            curva_nov_sinis=0;
            break;
        }
    }
    if(curva_nov_dest & curva_nov_sinis)
        return 3;
    else if(curva_nov_dest)
        return 1;
    else if(curva_nov_sinis)
        return 2;
    else
        return 0;
}

void curva90(char dir, int quanto = 20){

    if(dir=='d'){ ///CURVA A DESTRA
                for(int i=0;i<quanto;i++){
                    setSpeeds(30,30);
                    usleep(100000);
                }
                for(int i=0;i<20;i++){
                    setSpeeds(50,-50);
                    usleep(100000);
                }
                for(int i=0;i<5;i++){
                    setSpeeds(-30,-30);
                    usleep(150000);
                }
            }
            else if(dir=='s'){ ///CURVA A SINISTRA
                for(int i=0;i<20;i++){
                    setSpeeds(30,30);
                    usleep(100000);
                }
                for(int i=0;i<20;i++){
                    setSpeeds(-50,50);
                    usleep(100000);
                }
                for(int i=0;i<5;i++){
                    setSpeeds(-30,-30);
                    usleep(150000);
                }
            }
}

#define SINGLEFRAME 1

float errore_pan=1;
int errore_vero=-1;

int main(int argc, char** argv )
{


    #if GRAPHICAL
        namedWindow("Line",1);
        amedWindow("Original",1);
        //namedWindow("Elab",1);
    #endif
    VideoWriter video("video.avi",CV_FOURCC('M','J','P','G'),1,Size(160,120));

    fd = Serial_start();
    attach("pan");
    attach("tilt");
    setTilt(20);
    //namedWindow("immagine2.png",WINDOW_AUTOSIZE);
    #if SINGLEFRAME
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 160);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 120);
    #if GRAPHICAL
        namedWindow("colore",WINDOW_FULLSCREEN);
    #endif

    ifstream input("/sys/class/gpio/gpio4/value");
    int n;

    /// QUI INIZIA IL LOOP
    for(;;){
        ///QUI CONTROLLO SE POSSO ANDARE AVANTI O SONO STATO RESETTATO DA PULSANTE
        input.clear();
		input.seekg(0);
		input >> n;
		if(n==1){
            return 0;
		}
        #else
        frame=imread(argv[1], CV_LOAD_IMAGE_COLOR);
        if ( !frame.data )
        {
            printf("No image data \n");
            return -1;
        }
        #endif // SINGLEFRAME

        #if SINGLEFRAME
        for(int contrl=0;contrl<10;contrl++){
        cap >> frame; // get a new frame from camera
        lns=Mat::zeros( 160, 120, CV_8UC3 );
        }
        cout<<"Stampa"<<endl;

    #endif // SINGLEFRAME

        frame.copyTo(copiacolore);
        cvtColor(frame, edges, CV_BGR2GRAY);
        threshold(edges,thr, 70, 255, THRESH_BINARY);
        grigio=thr;
        cvtColor(grigio,colore, CV_GRAY2BGR);
        #if GRAPHICAL
            imshow("Original",frame);
            imshow("Line",thr);
            waitKey(30);
        #endif
		///erosion
        //TROVALINEA!
                //INIZIALIZZAZIONE

				distanze.clear();
				hd_copia.clear();
				hdng_isole.clear();
				while(!coda.empty())
					coda.pop();

				for(int i=0; i<10000; i++)
				{
				linea_x[i]=0;
				linea_y[i]=0;
				vettore_x[i].clear();
				vettore_y[i].clear();
				//vett_quanti[i]=0;
				}
				cvtColor(grigio,colore, CV_GRAY2BGR);

				bool gap = false;
				bool trovato_isole=false;
				bool trovato_verde=false;
				bool incrocio_t = false;
				bool trovato_linea=false;
				bool linea_corta=false;
				bool isole_corta = false;
				int direzione_principale;
				int quante_isole=0;
                int errore=0;
                char dove = 'z';
				Point verde;
				distanza_max=0;

        //FINZE INIZIALIZZAZIONE---------------
            #if DEBUG2
            cout<<"robe principali\n";
            imwrite("immagine2.png", colore);
            #endif // DEBUG2
        Erosion(0,0,grigio);
        Dilation(0,0,grigio);
        Dilation(0,0,grigio);
        trovato_linea = bfs_principaple(grigio,true);
        if(!trovato_linea){
            cout<<"\n----------SERIALE ALL ARDU: TUTTO BIANCO------------\n"<<endl;
            //goto fineframe;
        }

        fino_a_dove=distanza_max-1;
        if(distanza_max<20)
        {
            cout<<"-----SERIALE: TROPPO CORTA LINEA"; ///VAI AVANTI UN POCHETTO E RIFALLO?
            linea_corta = true;
            //goto fineframe;
        }
            #if DEBUG2
            cout<<"LA DISTANZA MASSIMA TROVATA :"<<distanza_max<<endl;
            //imshow("colore", colore);
            //waitKey();

            #endif // DEBUG2
        //GAP


            if(trovato_linea){
                    #if DEBUG2
                    cout<<"ricerca gap\n";
                    #endif // DEBUG2
                if(distanza_max>=5)
                if(condizione_di_gap(vettore_x[distanza_max-3][0],vettore_y[distanza_max-3][0])){
                    gap=true;
                    cout<<"\n----------SERIALE ALL ARDU: GAP------------\n"<<endl;
                        #if DEBUG2
                        cout<<"GAP \n"<<vettore_x[distanza_max-3][0]<<" "<<vettore_y[distanza_max-3][0]<<endl;
                        colore.at<Vec3b>(vettore_y[distanza_max-3][0],vettore_x[distanza_max-3][0]) = (Vec3b){	0, 102, 255};
                        #endif // DEBUG2
                    int base=vettore_y[distanza_max-3][0];
                    //METODO IGNORANTE!!!!!
                    base-=20;
                    pair<int,int> risultato=trova_primo_nero(base);
                    //while(risultato.second == 50 && base >= 50){ ///non serve : quanti gap ci possono essere in una sola immagine se non 2?????
                    while(base >=0  && risultato.first==-1){
                        base-=10;
                        risultato=trova_primo_nero(base);
                    }
                    bfs_principaple(grigio,false,risultato.first,risultato.second);
                    //goto fineframe;
            }
		//RICERCA ISOLE SUL PERCOROSO
                #if DEBUG2
                cout<<"ricerca isole + verde\n";
                #endif // DEBUG2

                    if(!linea_corta)
                    for(int dove=2; dove<fino_a_dove; dove+=50){
                        if(bfs_isole(grigio,dove)>=2){
                            fino_a_dove=dove;
                            trovato_isole=true;
                            break;
                         }
                    }
                    if(trovato_isole){
                    int ub=fino_a_dove;
                    int lb=2;
                    int mid=(ub+lb)/2;

                    while(ub>lb+1){
                        mid=(ub+lb)/2;
                        if(bfs_isole(grigio,mid)==1)
                            lb=mid;
                        else
                            ub=mid;
                    }
                        if(distanza_max - ub >=30)  ///se hai un po di spazio allarga pure, perchè sennò stai stretto;
                            distanza_isole=ub+10;
                        else
                            distanza_isole=ub;

                        fino_a_dove=distanza_isole;
                        quante_isole=bfs_isole(grigio,distanza_isole);
                        if(fino_a_dove <= 30){ ///ASPETTIAMO L'HEADING PER DIRE SE SI TRATTA DI UN INCROCIO A T, DIRITTO O A 4
                            linea_corta=true;

                        }
                        cout<<"\n----------SERIALE ALL ARDU: TROVATE ISOLE------------\n"<<endl;
                            #if DEBUG2
                            imwrite("immagine2.png", colore);
                            getchar();
                            cout<<"le isole le trovo a distanza : "<<distanza_isole<<endl;
                            cout<<"ci sono: "<<quante_isole<<"isole\n";
                            cout<<"INIZIO CALCOLO HEADING ISOLE\n";
                            #endif // DEBUG2

                    heading_isole(grigio);
                    /*FAI COSE PER DETERMINARE IL TIPO DI INCROCIO*/
                    #if DEBUG2
                    assert(hdng_isole.size()==quante_isole);
                    #endif // DEBUG2

                    for(int i=0; i<quante_isole; i++)
                    {
                        if(hdng_isole[i]==-5)
                            isole_corta=true;
                    }

                    }///endif trovato_isole
		//FINE RICERCA ISOLE SUL PERCORSO
            //CALCOLO CORREZIONI ROBOT UNA VOLTA TRATTATA LA LINEA PRINCIPALE
            #if DEBUG2
            cout<<"tratto linea principale \n";
            #endif // DEBUG2
            if(!linea_corta)
            {
                tratta_linea_principale();
                direzione_principale = angolo_medio_linea(linea_x,linea_y,fino_a_dove, 10,3);
                errore =calcola_errore(direzione_principale,linea_x[2]);
                last_errore=errore;
            }
            else
                incrocio_t = true;

            #if DEBUG2
            cout<<"angolo linea principale: "<<direzione_principale<<endl;
            //cout<<"\nl'errore e' "<<calcola_errore(direzione_principale,linea_x[2]);
            #endif // DEBUG2

            ///setSpeeds(30+(errore)/5,30-(errore)/5);
            cout<<"ERRORE: "<<errore<<endl;

            //verde
            if(trovato_isole){
                verde = trova_verde();
                if(verde.x != -1)
                    trovato_verde=true;
            }

            if(trovato_verde){
                dove=tratta_verde_linea(Point2i((int)verde.x, (int)verde.y), Point2i(linea_x[distanza_isole-5],linea_y[distanza_isole-5]));
                #if DEBUG2
                cout<<"verde x e y"<<verde.x<<" "<<verde.y<<"    "<<linea_x[distanza_isole-5]<<"   "<<linea_y[distanza_isole-5]<<endl;
                cout<<"IL VERDE EVENTUALMENTE TROVATO SI TROVA A "<<dove<<endl;
                #endif // DEBUG2
                }

            //cout<<"\n----------SERIALE ALL ARDU: linea normale------------\n"<<endl;
        } ///ENDIF TROVATO LINEA

        /**int curva_nov=check_curva_nov(); ///1 a destra, 2 a sinistra, 3 incrocio, 0 non ho trovato nulla
        //cout << grigio.rows << endl;
        if(curva_nov>0){    ///SE C'È QUALCOSA DA FARE PER VIA DI INCROCI/CURVE POTENTI
            if(curva_nov==1){ ///CURVA A DESTRA
                for(int i=0;i<20;i++){
                    setSpeeds(30,30);
                    usleep(100000);
                }
                for(int i=0;i<20;i++){
                    setSpeeds(50,-50);
                    usleep(100000);
                }
                for(int i=0;i<5;i++){
                    setSpeeds(-30,-30);
                    usleep(100000);
                }
            }
            else if(curva_nov==2){ ///CURVA A SINISTRA
                for(int i=0;i<20;i++){
                    setSpeeds(30,30);
                    usleep(100000);
                }
                for(int i=0;i<20;i++){
                    setSpeeds(-50,50);
                    usleep(100000);
                }
                for(int i=0;i<5;i++){
                    setSpeeds(-30,-30);
                    usleep(100000);
                }
            }
            else if(curva_nov==3){ ///INCROCIO, VAI DRITTO PER EVITARE ERRORI
                for(int i=0;i<20;i++){
                    setSpeeds(30,30);
                    usleep(100000);
                }
            }
        }
        else{
            if(direzione_principale<=30){
                errore_pan=1.4;
                setPan(70);
            }
            else if(direzione_principale<=45 && direzione_principale>35){
                errore_pan=1.2;
                setPan(80);
            }
            else if(direzione_principale>=150){
                errore_pan=1.4;
                setPan(100);
            }
            else if(direzione_principale>=135 && direzione_principale<125){
                errore_pan=1.2;
                setPan(120);
            }
            else if(direzione_principale>70){
                errore_pan=0.8;
                setPan(90);
            }


            if(errore_vero==-1){
                errore_vero=errore;
            }
            else{
                errore_vero=(errore_vero+errore*2)/3;
            }


            setSpeeds(30+((errore_vero)/3*errore_pan),30-((errore_vero)/3)*errore_pan);
            cout<<"ERRORE: "<<errore << " → " << errore_vero <<endl;
        }*/


         ///PARTE DECISIONALE OPERATIVA: CHE CAZZO FACCIO?
        #if DEBUGOP
        {
            cout<<"TROVATO LINEA?"<<trovato_linea<<endl;
            cout<<"LINEA CORTA?"<<linea_corta<<endl;
            cout<<"ISOLE?"<<trovato_isole<<" quante  "<<quante_isole<<" ce ne sono corte ? "<<isole_corta<<endl;
            for(int i=0; i<hdng_isole.size(); i++)
            {
                cout<<"hdng isola "<<i<<" "<<hdng_isole[i]<<endl;
            }
            cout<<"verde? "<<trovato_verde<<endl;
            imwrite("immagine2.png", colore);
        }
        #endif // DEBUGOP


        if(!trovato_linea || gap){
            setSpeeds(30+(last_errore)/5,30-(last_errore)/5);
            last_errore=last_errore/2;
            //FINISCI!
        }
        if(gap) ; ///FAI COSE, ANCORA DA DEFINIRE!!
        ///SE HAI TROVATO ISOLE STRANE MA UNA LINEA DECENTE, FREGACAZZI E SEGUI QUELLA
        if(trovato_isole && isole_corta && !linea_corta) trovato_isole = false;


        if(trovato_isole){
                int h1,h2,h3;
                sort(hdng_isole.begin(), hdng_isole.end());
                hd_copia=hdng_isole;
                for(int i=0; i<hdng_isole.size(); i++)
                {
                    if(compreso(hdng_isole[i],45,135)) hdng_isole[i]=90;
                    if(compreso(hdng_isole[i],136,225)) hdng_isole[i]=180;
                    if(compreso(hdng_isole[i],226,315)) hdng_isole[i]=270;
                    if(hdng_isole[i]!=-5 && (hdng_isole[i] <=45 || hdng_isole[i] >= 316)) hdng_isole[i]=0;

                }

                switch(quante_isole){
                    case 2: ///INCROCIO (?) DRITTO CON DUE ISOLE
                        h1 = hdng_isole[0];
                        h2 =hdng_isole[1];
                        if(h1 > h2) swap(h1,h2);

                        if(!trovato_verde){
                            if(h1==90 || h2 == 90 ){
                            cout<<"HO TROVATO UN INCROCIO (a 2) DA ANDARE DRITTO(senza verde)\n";
                            setSpeeds(30+(last_errore)/5,30-(last_errore)/5);
                            last_errore=last_errore/2;
                            }

                            if(h1==0 && h2==180){ ///INCROCIO A T, MORTA
                                cout<<"allarme: incrocio a T senza verde. torno indietro\n";
                                setSpeeds(-30,-30);
                                ///wait(100);?????'
                            }
                        }
                        else{ ///TROVATO VERDE!
                            if(dove == 's')
                                    curva90('s',linea_y[fino_a_dove]/20 + 20);
                                    ///vacci a sinistra!
                            if(dove == 'd')
                                 curva90('d',linea_y[fino_a_dove]/20 + 20);
                                ///vacci a dx!

                        }

                    break;
                    case 3:
                        h1 = hdng_isole[0];
                        h2 = hdng_isole[1];
                        h3 = hdng_isole[2];

                        if(!trovato_verde){
                            if(!isole_corta && h2==90 /*&& h1==0 && h3 == 180*/ && linea_corta){
                                ///SE SEI NELLE ISOLE CON LA LA LINEA CORTA ALLROA SEGUI QUELLO
                                cout<<"HO TROVATO UN INCROCIO ( a 3) DA ANDARE DRITTO(senza verde)\n";
                                errore = calcola_errore(hd_copia[1],linea_x[fino_a_dove],linea_y[fino_a_dove]); ///
                                last_errore=errore;
                                setSpeeds(30+(last_errore)/5,30-(last_errore)/5);
                                last_errore=last_errore/2;
                            }
                            /*if(isole_corta && !linea_corta){
                                ///SE HAI TROVATO ISOLE CORTE O VAI AVANTI NORMALE, O NON FARE UN CAZZO
                                setSpeeds(30+(last_errore)/5,30-(last_errore)/5);
                                last_errore=last_errore/2;
                            } E' GIA STATO TRATTATO ALL'INIZIO!!!*/
                            if(isole_corta && linea_corta){
                                cout<<"ALLARME: linea corta e isole corte. torno indietro\n";
                                setSpeeds(-30,-30);
                                ///wait(100);?????'
                            }
                            if(!isole_corta && !linea_corta && h2==90){
                            cout<<"incrocio tre, devo andare dritto ma sto a metà. magari gira la camera gotta, non soh";
                                errore = calcola_errore(hd_copia[1],linea_x[2],linea_y[2]);
                                last_errore=errore;
                                setSpeeds(30+(last_errore)/5,30-(last_errore)/5);
                                last_errore=last_errore/2;
                            }

                        }
                        else{ ///TROVATO VERDE
                            if(dove == 'e'){
                                cout<<"ALLARME: verde strano su inc. a 3\n";
                                setSpeeds(-30,-30);
                                ///?ruotare la camera?
                            }
                            //wait(distanza_isole); ///APPROPINQUATI ALL'INCROCIO, POI GIRA;
                            if(dove == 's'){
                                cout<<"incrocio a tre con verde : GIRO A SINISTRA \n";
                                curva90('s',linea_y[fino_a_dove]/20 + 20);
                            }
                            if(dove == 'd'){
                                cout<<"incrocio a tre con verde :GIIRO A DX \n";
                                curva90('d',linea_y[fino_a_dove]/20 + 20);
                            }
                        }

                    break; ///FINE INCROCIO A TRE

                }
        } //endif TROVATO_ISOLE

        if(!linea_corta){ ///SE NON HAI LA LINEA CORTA, ALLORA NELLA VARIABILE ERRORE STA SALVATO ERRORE. usarlo!
            if(direzione_principale > 145) curva90('s');
            else if(direzione_principale < 35) curva90('d');
            else{
                setSpeeds(30+(last_errore)/5,30-(last_errore)/5);
                last_errore=last_errore/2;
            }
        }


        //Fine movimento cose

        //cout<<"\n----------SERIALE ALL ARDU: linea normale------------\n"<<endl;
        ///ENDIF TROVATO LINEA
        #if DEBUG2

        cout<<"\n*************fine frame**************\n";
        video.write(colore);getchar();
        #endif // DEBUG2

        //Dilation(0,0);

        //imshow("Line", thr);
        //imshow("Elab", lns);
        //if(waitKey(30) >= 0) break;
        //waitKey(0);

    //fineframe:
    #if SINGLEFRAME
    }
    #endif // SINGLEFRAME
    // the camera will be deinitialized automatically in VideoCapture destructor

    cout<<"MEGA DEBUGGONE 22/4\n";
    cout<<"LINEAx E LINEAy  "<<linea_x[0]<<" "<<linea_y[0]<<linea_x[1]<<" "<<linea_y[1]<<endl;
    cout<<vettore_x[0].size()<<" "<<vettore_y[0].size()<<vettore_x[1].size()<<" "<<vettore_y[1].size()<<vettore_x[3].size()<<" "<<vettore_y[3].size()<<endl;



    return 0;
}




////MERDA!!



    /*
    namedWindow("Settings",0);
    createTrackbar( "Erosion --- Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Settings", &erosion_elem, max_elem);
    createTrackbar( "Erosion --- Kernel size:\n 2n +1", "Settings", &erosion_size, max_kernel_size);
    createTrackbar( "Dilation --- Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Settings", &dilation_elem, max_elem);
    createTrackbar( "Dilation --- Kernel size:\n 2n +1", "Settings", &dilation_size, max_kernel_size);
    */

                            /**
///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<VECCHIO>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                        if(h1==90 || h2 == 90 && (h1!=-5 && h2!=-5)) ///INCROCIO DRITTO CON BRACCIO DRITTO (e l'altro non si sa???wtf?)
                        {
                            ///fai cose,vai dritto? non so
                            if(!trovato_verde){
                            cout<<"HO TROVATO UN INCROCIO (a 2) DA ANDARE DRITTO(senza verde)\n";
                            setSpeeds(30+(last_errore)/5,30-(last_errore)/5);
                            last_errore=last_errore/2;
                            }
                            else
                            {
                                if(dove == 's'){
                                    cout<<"HO TROVATO UN INCROCIO DA ANDARE A SX\n";
                                    ///vacci a sinistra!
                                }
                                if(dove == 'd'){
                                    cout<<"HO TROVATO UN INCROCIO DA ANDARE A DX\n";
                                    ///vacci a dx!
                                }
                            }
                        }
                        if(h1==0 && h2==180) ///INCROCIO A T, MORTA
                        {
                            if(!trovato_verde){
                            cout<<"allarme: incrocio a T senza verde. torno indietro\n";
                            setSpeeds(-30,-30);
                            ///wait(100);?????'
                            }

                            if(dove == 's'){
                                    curva90('s',20);
                                    ///vacci a sinistra!
                                }
                                if(dove == 'd'){
                                     curva90('d',20);
                                    ///vacci a dx!
                                }
                        }

        ///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<VECCHIO>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        */

