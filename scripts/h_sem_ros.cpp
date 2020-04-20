#include <opencv2/core.hpp>
#include <bits/stdc++.h>
using namespace cv;
using namespace std;

#define vp vector<pair<double, double>>

vp order_points(vp pts){

    sort(pts.begin(), pts.end());

    pair<double, double> bl;

    if(pts[0].second > pts[1].second){
        bl = pts[1];
    }else{
        bl = pts[0];
        pts[0] = pts[1];
    }
    
    if(pts[2].second < pts[3].second){
        pts[1] = pts[3];
    }else{
        pts[1] = pts[2];
        pts[2] = pts[3];
    }
    pts[3] = bl;

    return pts;
}

int main(){

    vector<pair<double, double>> test, result; 
    test = { make_pair(0,1), make_pair(2,3), make_pair(4,5), make_pair(6,7)};

    result = order_points(test);

    for(auto loop : result) cout<<loop.first<<" "<<loop.second<<"\n";

    return 0;
}
