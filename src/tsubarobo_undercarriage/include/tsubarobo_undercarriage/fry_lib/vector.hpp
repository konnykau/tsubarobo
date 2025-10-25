#pragma once
namespace FRY{
    struct vec2d{
        float x;
        float y;
        //ベクトル成分
        friend auto operator*(const vec2d a,const vec2d b) -> float{
            return a.x*b.x+b.y*a.y;
        }//内積
        static vec2d make(float x,float y){
            return vec2d{.x = x,.y = y};
        }//ｘ,ｙを要素にもつベクトルを返す
    };
    
}