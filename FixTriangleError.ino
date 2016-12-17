//fix the shorter side if a triangle
//is not able to be formed
float adjust(float Sl,float Ss)
{
  float c = 18.75;
  if (Sl >= (Ss+c)){
  float extra = Sl-(Ss+c)+1;
  Ss = Ss+extra;}
  return Ss;
}

