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
