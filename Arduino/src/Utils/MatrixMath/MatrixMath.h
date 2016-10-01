/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Matrix math
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
 */

 #ifndef MATRIXMATH_H
 #define MATRIXMATH_H

 // vector
 void Vector_Cross_Product(float saida[3], const float v1[3], const float v2[3]);
 void Vector_Add(float saida[3], const float v1[3], const float v2[3]);
 void Vector_Scale(float saida[3], const float v[3], const float scale);
 float Vector_Dot_Product(const float v1[3], const float v2[3]);

 // matrix
 void Matrix_Multiply(const float a[3][3], const float b[3][3], float saida[3][3]);


 #endif
