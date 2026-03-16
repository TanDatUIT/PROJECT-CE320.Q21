#include <stdio.h>
#include <math.h>
#include <Fuzzy.h>
#include <Arduino.h>
// Fuzzy logic for image processing using edge detection
// Define the fuzzy sets for edge detection
// Định nghĩa tập mờ cho phát hiện cạnh
 
/*Mặt nạ 2x2 [ Tối ưu cho esp32]
đầu vào: 4 pixel [ P1, P2, P3, P4]
tập mờ input : 2 tập black and white dùng hàm thuộc hình tam giác
tập mờ output: black, edge và white
luật mờ: 16 luật [ tài liệu 2010]
*/
