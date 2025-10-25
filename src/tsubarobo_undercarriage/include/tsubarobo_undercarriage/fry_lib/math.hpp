#pragma once
namespace FRY{
    template <typename T>
    constexpr float sqrt(T x){
        constexpr float error_range = 0.000001;
        if(x > 1){
            float approximation = x;
            while(approximation*approximation > x + error_range){
            approximation = (approximation*approximation + x)/(2*approximation);
            }
            return approximation;
        }
        if(x <= 0)return 0;
        if(x <= 1){
            float approximation = 1.0;
            while(approximation*approximation > x + error_range){
            approximation = (approximation*approximation + x)/(2*approximation);
            }
            return approximation;
        }

        return 0;

    }
}