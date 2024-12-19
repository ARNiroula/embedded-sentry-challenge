#include <cstring>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <string>

#include "dtw_distance.hpp"

    /**
     * Compute the p_norm between two 1D c++ vectors.
     *
     * The p_norm is sometimes referred to as the Minkowski norm. Common
     * p_norms include p=2.0 for the euclidean norm, or p=1.0 for the
     * manhattan distance. See also
     * https://en.wikipedia.org/wiki/Norm_(mathematics)#p-norm
     *
     * @a 1D vector of m size, where m is the number of dimensions.
     * @b 1D vector of m size (must be the same size as b).
     * @p value of norm to use.
     */

float p_norm(float a[3], float b[3], uint32_t a_b_dimLEN, float p) {
    float d = 0;
    for (uint32_t i = 0; i < a_b_dimLEN; i++) {
        d += std::pow(std::abs(a[i] - b[i]), p);
    }
    return std::pow(d, 1.0 / p);
};

float d[MAX_ARRY_2D_SIZE][MAX_ARRY_2D_SIZE] = { 0 };
/**
 * Compute the DTW distance between two 2D c++ vectors.
 *
 * The c++ vectors can have different number of data points, but must
 * have the same number of dimensions. This will raise
 * std::invalid_argument if the dimmensions of a and b are different.
 * Here the vectors should be formatted as
 * [number_of_data_points][number_of_dimensions]. The DTW distance can
 * be computed for any p_norm. See the wiki for more DTW info.
 * https://en.wikipedia.org/wiki/Dynamic_time_warping
 *
 * @a 2D vector of [number_of_data_points][number_of_dimensions].
 * @b 2D vector of [number_of_data_points][number_of_dimensions].
 * @p value of p_norm to use.
 */
float dtw_distance_only(float a[MAX_ARRY_2D_SIZE][3], uint32_t a_dataPointLEN, uint32_t a_dimLEN,
    float b[MAX_ARRY_2D_SIZE][3], uint32_t b_dataPointLEN, uint32_t b_dimLEN, float p)

{
    uint32_t n = a_dataPointLEN;
    uint32_t o = b_dataPointLEN;
    uint8_t a_m = a_dimLEN;
    uint8_t b_m = b_dimLEN;
    if (a_m != b_m)
    {
        return -1;
        //throw std::invalid_argument("a and b must have the same number of dimensions!");
    }
    //std::vector<std::vector<float> > d(n, std::vector<float>(o, 0.0));

    //d is 2d size [a_dataPointLEN by][b_dataPointLEN]

    // float d[MAX_ARRY_2D_SIZE][MAX_ARRY_2D_SIZE] = { 0 };
    memset(d, 0, sizeof d);
    d[0][0] = p_norm(a[0], b[0], a_dimLEN, p);    //in production, can hardcode a_dimLEN = 3
    for (uint32_t i = 1; i < n; i++)
    {
        d[i][0] = d[i - 1][0] + p_norm(a[i], b[0], a_dimLEN, p);
    }
    for (uint32_t i = 1; i < o; i++)
    {
        d[0][i] = d[0][i - 1] + p_norm(a[0], b[i], a_dimLEN, p);
    }
    for (uint32_t i = 1; i < n; i++)
    {
        for (uint32_t j = 1; j < o; j++) {
            d[i][j] = p_norm(a[i], b[j], a_dimLEN, p) + std::fmin(std::fmin(d[i - 1][j], d[i][j - 1]), d[i - 1][j - 1]);
        }
    }
    return d[n - 1][o - 1];
};



// int main ()
// {
//   float p = 2;  // the p-norm to use; 2.0 = euclidean, 1.0 = manhattan
//   std::vector<std::vector<float> > a = { {0, 2}, {1, 2}, {2, 2}, {3, 2}, {4, 2} };
//   std::vector<std::vector<float> > b = { {0, 2}, {1, 1}, {2, 2}, {3, 1}, {4, 2} };

//   float a_embed[MAX_ARRY_2D_SIZE][3] = { 0 };
//   float b_embed[MAX_ARRY_2D_SIZE][3] = { 0 };

//   //use the vects to make embed vers
//   for (int i = 0; i < a.size(); i++)
//   {
// 	  for (int j = 0; j < a[0].size(); j++)
// 	  {
// 		  a_embed[i][j] = a[i][j];
// 	  }
//   }

//   for (int i = 0; i < b.size(); i++)
//   {
// 	  for (int j = 0; j < b[0].size(); j++)
// 	  {
// 		  b_embed[i][j] = b[i][j];
// 	  }
//   }


  // Compute the DTW distance between a an b
  // a[number_of_data_points][number_of_dimensions]
  // b[number_of_data_points][number_of_dimensions]
  // The number of dimensions between a and b must agree
  //for deployment, make sure that 
  //    float d[MAX_ARRY_2D_SIZE][MAX_ARRY_2D_SIZE] = { 0 };
  //    
  //    is declared outside of main, so that it's allocated
  //    in stack away from user code 
  //    (doing so otherwise crashes due to the stack ramming into program stack code)

//   std::cout << "DTW distance [embed]: " << dtw_distance_only(a_embed, a.size(), a[0].size(), b_embed, b.size(), b[0].size(), p) << std::endl;
//   return 0;
// }