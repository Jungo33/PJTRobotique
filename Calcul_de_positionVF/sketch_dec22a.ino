#include <stdio.h>
#include "math.h"

float x,y;


void position(float alpha,float beta,float gama)      //Fonction de calcul de la position en fonction des angles inter balise
{
    //Dimenssions et angles connus
    float L = 2040; // longueur du terrain
    float w = 1660;  // longueur entre B1 et B2 = 2 x 830 mm
    float v = sqrt(pow(830,2)+ pow(2040,2)); // longueur entre B1 et B3
    float u = sqrt(pow(830,2)+ pow(2040,2)); // logueur entre B2 et B3
    float r = atan(830/2040);  // angle(0,B1,B3)
    float q = (M_PI/2)-r;  // angle(B1,B3,0)
    float s = atan(830/2040); //angle
    float t = (M_PI/2)-s;  //angle
    
    // Calcul des angles inconnus
    float j = acos(sqrt(pow(((u/sin(beta))*cos(s-beta+M_PI/2)+(w/sin(alpha))*cos(alpha-M_PI)),2)/(pow(((u/sin(beta))*cos(s-beta+M_PI/2)+(w/sin(alpha))*cos(alpha-M_PI)),2)+pow(((u/sin(beta))*sin(s-beta+M_PI/2)+(w/sin(alpha))*sin(alpha-M_PI)),2))));
    float i = M_PI-alpha-j;
    printf("%.6f", j);
    printf("\n");
    printf("%.6f", i);
    printf("\n");
    
    //Calcul des longueurs inconnues
    float a =  w*sin(j)/sin(alpha);
    float b = w*sin(i)/sin(alpha);
    printf("%.6f", a);
    printf("\n");
    printf("%.6f", b);
    printf("\n");
   
    //Calcul des coordonés
    x = cos(i)*a;
    y = L - sin(i)*a;
}


int main(int argc, const char * argv[]) {
    
    position(M_PI/2,3*M_PI/4,3*M_PI/4); //Fonction de calcul de la position
    printf("%.6f", x);
    printf("\n");
    printf("%.6f", y);
    printf("\n");
    
    return 0;
}



