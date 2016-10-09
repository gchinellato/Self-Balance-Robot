/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Matrix math
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
 */

 void Vector_Cross_Product(float saida[3], const float v1[3], const float v2[3])
 {
 	saida[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
 	saida[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
     saida[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
 }

 void Vector_Add(float saida[3], const float v1[3], const float v2[3])
 {
 	for (int c = 0; c < 3; c++)
 	{
 		saida[c] = v1[c] + v2[c];
 	}
 }

 void Matrix_Multiply(const float a[3][3], const float b[3][3], float saida[3][3])
 {
 	for (int x = 0; x < 3; x++)  // rows
 	{
 		for (int y = 0; y < 3; y++)  // columns
 		{
 			saida[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
 		}
 	}
 }

 float Vector_Dot_Product(const float v1[3], const float v2[3])
 {
 	float result = 0;

 	for (int c = 0; c < 3; c++)
 	{
 		result += v1[c] * v2[c];
 	}

 	return result;
 }

 void Vector_Scale(float saida[3], const float v[3], const float scale)
 {
 	for (int c = 0; c < 3; c++)
 	{
 		saida[c] = v[c] * scale;
 	}
 }
