#ifndef MISC_UTILS_HPP
#define MISC_UTILS_HPP

#include <bits/stdc++.h>
#include "utils/utils.hpp"
namespace Planners
{
    namespace Misc
    {   
        //Found in 
        //https://www.codespeedy.com/hsv-to-rgb-in-cpp/
        /**
         * @brief 
         * 
         * @param _H 
         * @param _S 
         * @param _V 
         * @return utils::Vec3i 
         */
        utils::Vec3i HSVtoRGB(float _H,const float _S,const float _V)
        {
            if (_S > 100 || _S < 0 || _V > 100 || _V < 0)
            {
                std::cout << "The givem SV values are not in valid range" << std::endl;
                std::cout << "HSV: [" << _H << ", " << _S << ", " << _V << " ]" << std::endl;
                return {};
            }
            while ( _H > 360)
                _H -= 360;
            
            while( _H < 0 )
                _H += 360;
            
            float s = _S / 100;
            float v = _V / 100;
            float C = s * v;
            float X = C * (1 - std::abs(std::fmod(_H / 60.0, 2) - 1));
            float m = v - C;
            float r, g, b;
            if (_H >= 0 && _H < 60)
            {
                r = C, g = X, b = 0;
            }
            else if (_H >= 60 && _H < 120)
            {
                r = X, g = C, b = 0;
            }
            else if (_H >= 120 && _H < 180)
            {
                r = 0, g = C, b = X;
            }
            else if (_H >= 180 && _H < 240)
            {
                r = 0, g = X, b = C;
            }
            else if (_H >= 240 && _H < 300)
            {
                r = X, g = 0, b = C;
            }
            else
            {
                r = C, g = 0, b = X;
            }
            int R = (r + m) * 255;
            int G = (g + m) * 255;
            int B = (b + m) * 255;

            return utils::Vec3i{R, G, B};
        }
    }
}

#endif