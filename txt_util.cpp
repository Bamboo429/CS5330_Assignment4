//
// Created by Chu-Hsuan Lin on 2022/3/16.
//
#include <cstdio>
#include <iostream>
#include <vector>

int getstring( FILE *fp, char os[] ) {
    int p = 0;
    int eol = 0;

    for(;;) {
        char ch = fgetc( fp );
        if( ch == ',' ) {
            break;
        }
        else if( ch == '\n' || ch == EOF ) {
            eol = 1;
            break;
        }
        // printf("%c", ch ); // uncomment for debugging
        os[p] = ch;
        p++;
    }
    // printf("\n"); // uncomment for debugging
    os[p] = '\0';

    return(eol); // return true if eol
}

int getfloat(FILE *fp, float *v) {
    char s[256];
    int p = 0;
    int eol = 0;

    for(;;) {
        char ch = fgetc( fp );
        if( ch == ',') {
            break;
        }
        else if(ch == '\n' || ch == EOF) {
            eol = 1;
            break;
        }

        s[p] = ch;
        p++;
    }
    s[p] = '\0'; // terminator
    *v = atof(s);

    return(eol); // return true if eol
}

int readInstrinsic(char* filename, std::vector<float> &camera_matrix, std::vector<float> &dist_coef){

    char buffer[256];
    float fval;
    //std::vector<float> dvec;

    FILE *file;
    file=fopen(filename,"r");

    getstring(file,buffer);
    for(;;) {
        // get next feature
        float eol = getfloat( file, &fval );
        camera_matrix.push_back( fval );
        if( eol ) break;
    }

    getstring(file,buffer);
    for(;;) {
        // get next feature
        float eol = getfloat( file, &fval );
        dist_coef.push_back( fval );
        if( eol ) break;
    }

    return 0;

}



