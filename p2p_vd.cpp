int TwoPointPosePartialRotationDirect(const std::vector<Eigen::Vector3d>& model_point,
                                const std::vector< Eigen::Matrix<double,6,1> >& plucker_line,
                                Eigen::Matrix3d  soln_rotations[2],
                                Eigen::Vector3d  soln_translations[2]) {
  // we assume that the gravity direction is [0 0 1]'
  // if is not, you need to warp your coordinate system 
  
  static const double kEpsilon = 1e-9;
  // ensure the ray is normilised
  DCHECK_LT(fabs(plucker_line[1].block<3,1>(0,0).squaredNorm() - 1.0), kEpsilon);
  DCHECK_LT(fabs(plucker_line[2].block<3,1>(0,0).squaredNorm() - 1.0), kEpsilon);

  // construct matrix M
  double M[2][12];
  for (int i = 0; i < 2; ++i)
  {
    M[i][0] = -plucker_line[i](2);
    M[i][1] = plucker_line[i](1);
    M[i][2] = model_point[i](1)*plucker_line[i](2);
    M[i][3] = -2.0*model_point[i](0)*plucker_line[i](2);
    M[i][4] = -model_point[i](1)*plucker_line[i](2);
    M[i][5] = model_point[i](2)*plucker_line[i](1) + plucker_line[i](4)*plucker_line[i](2) - plucker_line[i](5)*plucker_line[i](1);
    M[i][6] = plucker_line[i](2);
    M[i][7] = -plucker_line[i](0);
    M[i][8] = -model_point[i](0)*plucker_line[i](2);
    M[i][9] = -2.0*model_point[i](1)*plucker_line[i](2);
    M[i][10] = model_point[i](0)*plucker_line[i](2);
    M[i][11] = plucker_line[i](5)*plucker_line[i](0)-plucker_line[i](3)*plucker_line[i](2)-model_point[i](2)*plucker_line[i](0);
  }

  // construct the coefficients of the quadric equation
  // Denominator
  // 
  double De0 = M[0][1]*M[1][0]-M[0][0]*M[1][1];
  DCHECK_LT(fabs(De0), kEpsilon);
  double De1 = M[0][7]*M[1][6]-M[0][6]*M[1][7];
  DCHECK_LT(fabs(De1), kEpsilon);

  // numerator
  double Num0 = M[0][4]*M[1][0];
  double Num1 = M[0][0]*M[1][4];
  double Num2 = M[0][5]*M[1][0];
  double Num3 = M[0][0]*M[1][5];
  double Num4 = M[0][10]*M[1][6];
  double Num5 = M[0][6]*M[1][10];
  double Num6 = M[0][11]*M[1][6];
  double Num7 = M[0][6]*M[1][11];
  double Num8 = M[0][3]*M[1][0];
  double Num9 = M[0][0]*M[1][3];
  double Num10 = M[0][9]*M[1][6];
  double Num11 = M[0][6]*M[1][9];
  double Num12 = M[0][2]*M[1][0];
  double Num13 = M[0][0]*M[1][2];
  double Num14 = M[0][8]*M[1][6];
  double Num15 = M[0][6]*M[1][8];

  // composite
  double A,B,C;

  A = (Num0-Num1+Num2-Num3)/De0-(Num4-Num5+Num6-Num7)/De1;
  B = (Num8-Num9)/De0-(Num10-Num11)/De1;
  C = (Num12-Num13+Num2-Num3)/De0-(Num14-Num15+Num6-Num7)/De1;


  // solve the quadric equation
  double delt = B*B-4*A*C;
  if (delt < 0.0) // complex roots
  {
    return false;
  }

  double scale[2];
  scale[0] = (-B+sqrt(delt))/(2.0*A);
  scale[1] = (-B-sqrt(delt))/(2.0*A);


// maximum 2 solution
  int num_solutions = 0;
  for (int i = 0; i < 2; ++i)
  {
    double scaleSquarePlus1 = 1.0+scale[i]*scale[i];
    double scaleSquare = scaleSquarePlus1-1.0;
    double scaleSquareMinus1 = (scale[i]*scale[i]-1.0)/scaleSquarePlus1;
    double twoPlusscale = 2.0*scale[i]/scaleSquarePlus1;

    Eigen::Matrix3d Rotation;
    Rotation << scaleSquareMinus1, -twoPlusscale, 0,
                         twoPlusscale, scaleSquareMinus1,0,
                         0,0,1;

    double D[4];
    D[0] =  -((M[0][4]+M[0][5])*scaleSquare+M[0][3]*scale[i]+M[0][2]+M[0][5])/scaleSquarePlus1;
    D[1] =  -((M[0][10]+M[0][11])*scaleSquare+M[0][9]*scale[i]+M[0][8]+M[0][11])/scaleSquarePlus1,
    D[2] =  -((M[1][4]+M[1][5])*scaleSquare+M[1][3]*scale[i]+M[1][2]+M[1][5])/scaleSquarePlus1,
    D[3] =  -((M[1][10]+M[1][11])*scaleSquare+M[1][9]*scale[i]+M[1][8]+M[1][11])/scaleSquarePlus1;

    double t1,t2,t3;

    t3 = ((D[0]/M[0][0]-D[2]/M[1][0])/(M[0][1]/M[0][0]-M[1][1]/M[1][0]) + (D[1]/M[0][6]-D[3]/M[1][6])/(M[0][7]/M[0][6]-M[1][7]/M[1][6]))/2.0;
    t2 = ((D[0]-M[0][1]*t3)/M[0][0]+ (D[2]-M[1][1]*t3)/M[1][0])/2.0;
    t1 = ((D[1]-M[0][7]*t3)/M[0][6] + (D[3]-M[1][7]*t3)/M[1][6])/2.0;

    Eigen::Vector3d Translation(t1,t2,t3);


    bool inlier = true;

    for (int j = 0; j < 2; ++j)
    {
      double depth = plucker_line[j].block<3,1>(0,0).transpose()
                     *(Rotation*model_point[j]+Translation-plucker_line[j].block<3,1>(3,0));

      if (depth < 0.0)
      {
        inlier = false;
        break;
      }
    }

    if (inlier)
    {
      soln_rotations[num_solutions] = Rotation;
      soln_translations[num_solutions] = Translation;
      ++num_solutions;

    }

  }

  return num_solutions;
}
