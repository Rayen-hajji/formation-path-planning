#pragma once

#include <iostream>
#include <stack>
#include <algorithm>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

using namespace std;
using point = geometry_msgs::Point;
using polygon = vector<point>;


namespace formation_layer_namespace{
   geometry_msgs::Point p0; 

   point secondTop(stack<point> &stk) {
      point tempPoint = stk.top(); 
      stk.pop();
      point res = stk.top();    
      stk.push(tempPoint);      
      return res;
   }

   int squaredDist(point p1, point p2) {
      return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
   }

   //small value near zeroas follows :
   double eps = 1e-8;
   int direction(point a, point b, point c) {
      double val = (b.y-a.y)*(c.x-b.x)-(b.x-a.x)*(c.y-b.y);
      
      if (abs(val) < eps)
         return 0;    //colinear
      else if(val < 0)
         return 2;    //anti-clockwise direction
         return 1;    //clockwise direction
   }

   int comp(const void *point1, const void*point2) {
      point *p1 = (point*)point1;
      point *p2 = (point*)point2;
      int dir = direction(p0, *p1, *p2);
      if(dir == 0)
         return (squaredDist(p0, *p2) >= squaredDist(p0, *p1))?-1 : 1;
      return (dir==2)? -1 : 1;
   }

   polygon findConvexHull(polygon& points, int n) {
      polygon convexHullPoints;
      double minY = points[0].y, min = 0;
      
      for(int i = 1; i<n; i++) {
         double y = points[i].y;
         //find bottom most or left most point
         if((y < minY) || (minY == y) && points[i].x < points[min].x) {
            minY = points[i].y;
            min = i;
         }
      }

      swap(points[0], points[min]);    
      p0 = points[0];
      qsort(&points[1], n-1, sizeof(point), comp);    
      int arrSize = 1;    
      
      for(int i = 1; i<n; i++) {
         //when the angle of ith and (i+1)th elements are same, remove points
         while(i < n-1 && direction(p0, points[i], points[i+1]) == 0)
            i++;
            points[arrSize] = points[i];
            arrSize++;
      }

      if(arrSize < 3)
         return convexHullPoints;    //there must be at least 3 points, return empty list.
         //create a stack and add first three points in the stack
         stack<point> stk;
         stk.push(points[0]); stk.push(points[1]); stk.push(points[2]);
      
      for(int i = 3; i<arrSize; i++) {   
         while(direction(secondTop(stk), stk.top(), points[i]) != 2)
            stk.pop();    //when top, second top and ith point are not making left turn, remove point
            stk.push(points[i]);
      }
      
      while(!stk.empty()) {
         convexHullPoints.push_back(stk.top());    
         stk.pop();
      }
      return convexHullPoints;
   }
}