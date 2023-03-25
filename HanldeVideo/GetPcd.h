#pragma once
#include <stdio.h>
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <iostream>
#include <stdlib.h>
#include <k4a/k4a.hpp>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Python.h>
using namespace std;

void PyTxt2Pcd(string name_txt, string name_pcd);
int Video2Txt(const char* path, int start_second);