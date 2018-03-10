

# Lambda_zmp_walk-STM32F4
This project is walk pattern generate programs of the bipedal robot.
The means of walk pattern generation uses a dynamics filter using online pattern generation of the walk pattern generation to assume ZMP placed in the written by Shuuji Kajita "[humanoid robot][]" a model and the foresee control.

[humanoid robot]: https://www.ohmsha.co.jp/book/9784274200588/ "Humanoid robot"

"The humanoid robot" is the splendid book which covered all the basics of bipedal robot development, but it suddenly becomes difficult from a chapter of walking pattern generate which assumed ZMP a model, and understanding is difficult for a person of guide.I show it as an implementation.In addition, it is the publication only for numerical formulas in a book, but thinks that I may serve as a reference as an implementation of the practical use (consecutive walks and direction conversion etc..).

# License
MIT License

Copyright (c) 2018 Kenji Shimada

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

# Lambda
![Lambda](lambda_20180309.JPG)

Lambda is small bipedal robot developed for the study of the walk.
The actuator uses a servomotor for hobby robots.
The foot joint structure of the lambda does special structure.
I introduce joint structure of the lambda by the maintenance of the future document.

# Usage
You perform the operation of the robot from a console connected to UART.
The communication with the higher CPU uses this UART, too. When The Robot stoods up, he confirms specific FLASH domain and judge it whether he is connected to the CPU, or he is connected to the console.

At first you switch of the lower body (in other words it is both legs) of the robot.

\> lbp default 1 20 : robot power on lower body at default parameters.
 
 or
 
\> lbp 106.8 0 320 0 1 20 : robot power on lower body at setting paremeters. width=106.8mm depth=0mm height=320mm hung_at_freeleg=0mm center of mass adjustment=ON servo moving speed=20cycles(0.02sec/cycle)
 
Next, you switch of the upper body (it's both arms) of the robot.

\> ubp default 20
 
Next, you let you walk the robot.

\> zmp 0  : robot walk at default parameters.
 
 or
 
\> zmp 0 70 0.5 10  : robot walk at setting parameters mode=0(strate) stride=70mm pace=0.5sec steps=10steps 
 
 When you want to continue a walk, 
 
\> cont ( or c)

 or
 
\> cont 1 : curve=1

 or
 
\> cont 1 70 0.4 10 : curve stride=70mm pace=0.4sec steps=10steps

When you want to stop a walk,

\> stop ( or s)

