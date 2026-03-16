
/*Mặt nạ 2x2 [ Tối ưu cho esp32]
đầu vào: 4 pixel [ P1, P2, P3, P4]
tập mờ input : 2 tập black and white dùng hàm thuộc hình tam giác
tập mờ output: black, edge và white
luật mờ: 16 luật [ tài liệu 2010]
*/
// FOR GITHUB only edge detection not fuzzy logic in this file
/*
Edge detection algorithm
1. Firstly, take a colour image.
2. Refining: Refining is used to remove the noise as possible
without the damage of the true edges of it.
3. Intensification: Apply differentiation to enhance the quality of
edges.
4. Threshold: Edge magnitude threshold is used to reject the noisy
edge pixels and other should be confined.
5. Localization: Some applications to estimate the location of an
edge and spacing between pixels, sub pixels resolution might
be required.
6. Get the image after edge exposures
*/
// Lấy dữ liệu từ XuLyAnh.cpp để xử lý mờ
#include <stdio.h>
#include <math.h>
#include "XuLyAnh.h"

