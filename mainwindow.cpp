/***************************************************************************
**                                                                        **
**  QCustomPlot, an easy to use, modern plotting widget for Qt            **
**  Copyright (C) 2011-2018 Emanuel Eichhammer                            **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see http://www.gnu.org/licenses/.   **
**                                                                        **
****************************************************************************
**           Author: Emanuel Eichhammer                                   **
**  Website/Contact: http://www.qcustomplot.com/                          **
**             Date: 25.06.18                                             **
**          Version: 2.0.1                                                **
****************************************************************************/

/************************************************************************************************************
**                                                                                                         **
**  This is the example code for QCustomPlot.                                                              **
**                                                                                                         **
**  It demonstrates basic and some advanced capabilities of the widget. The interesting code is inside     **
**  the "setup(...)Demo" functions of MainWindow.                                                          **
**                                                                                                         **
**  In order to see a demo in action, call the respective "setup(...)Demo" function inside the             **
**  MainWindow constructor. Alternatively you may call setupDemo(i) where i is the index of the demo       **
**  you want (for those, see MainWindow constructor comments). All other functions here are merely a       **
**  way to easily create screenshots of all demos for the website. I.e. a timer is set to successively     **
**  setup all the demos and make a screenshot of the window area and save it in the ./screenshots          **
**  directory.                                                                                             **
**                                                                                                         **
*************************************************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <a_star.h>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);  
  setupGrid(ui->customPlot);
  ui->customPlot->replot();
}

void MainWindow::setupGrid(QCustomPlot *customPlot)
{
    // Define start and goal locations
    float startX = 10.0;
    float startY = 10.0;
    float goalX = 50.0;
    float goalY = 50.0;

    float gridSize = 1.0;
    float vehicleSize = 1.0;
    int pathLength = 0;

    // Create edges and obstacles
    vector<float> obsX;
    vector<float> obsY;

    for(float i = 0; i < 60; i++)
    {
        obsX.push_back(i);
        obsY.push_back(60.0);
    }
    for(float i = 0; i < 60; i++){
        obsX.push_back(60.0);
        obsY.push_back(i);
    }
    for(float i = 0; i < 61; i++){
        obsX.push_back(i);
        obsY.push_back(60.0);
    }
    for(float i = 0; i < 61; i++){
        obsX.push_back(0.0);
        obsY.push_back(i);
    }
    for(float i = 0; i < 40; i++){
        obsX.push_back(20.0);
        obsY.push_back(i);
    }
    for(float i = 0; i < 40; i++){
        obsX.push_back(40.0);
        obsY.push_back(60.0 - i);
    }
    runAStar(startX, startY, goalX, goalY, obsX, obsY, gridSize, vehicleSize, customPlot, pathLength);

    customPlot->xAxis->setRange(0, 65);
    customPlot->yAxis->setRange(0, 65);

    QCPItemText *caption = new QCPItemText(customPlot);
    caption->setPen(Qt::SolidLine);
    caption->setFont(QFont("Sans", 12, QFont::Thin));
    caption->setText("Red: start, Green: goal, Yellow: explored nodes, Blue: optimal path");
    caption->setTextAlignment(Qt::AlignCenter);    //center text within its rectangle
    caption->setPositionAlignment(Qt::AlignHCenter | Qt::AlignTop);    // use the center top of the rectangle to position it
    caption->position->setType(QCPItemPosition::ptAxisRectRatio);
    caption->position->setCoords(0.5, 0.02);
    caption->setClipToAxisRect(false);

    // Display path length
    QCPItemText *displayLength = new QCPItemText(customPlot);
    displayLength->setPen(Qt::SolidLine);
    displayLength->setFont(QFont("Sans", 12, QFont::Thin));
    displayLength->setText("Path Length: " + QString::number(pathLength));
    displayLength->setTextAlignment(Qt::AlignCenter);
    displayLength->setPositionAlignment(Qt::AlignHCenter | Qt::AlignTop);
    displayLength->position->setType(QCPItemPosition::ptAxisRectRatio);
    displayLength->position->setCoords(0.7, 0.8);
    displayLength->setClipToAxisRect(false);
}

MainWindow::~MainWindow()
{
  delete ui;
}



