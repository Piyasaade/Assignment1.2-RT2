Assignment 1
=============
This assignment is composed of three parts:

-The first part is: docummenting the third assignment of research track 1 using Sphinx or Doxygen.

-The second part is: creating a jupyter notebook as a user interface for the same assignmnet that can drive the robot.

-The third part is: statistical studies that is done to compare between my first assignment of research track one and the given from the Professor using specific tests.

First Part:
----------
Since my previous code was written using python, so in this part sphinx was better to use.
Sphinx was originally created for Python, but it has now facilities for the documentation of software projects in a range of languages.
where I commented my code by commenting each function i used what were the returns and the input for each function and by adding as an introduction for each .py file and at the end after applying all these steps we generate it as HTML file.

Second Part:
------------
In this part, a jupyter notebook is used, The Jupyter Notebook is an open source web application that you can use to create and share documents that contain live code, equations, visualizations, and text.
The main purpose of using this source is to create our userinterface using widgets to create our own buttons and sliders.
In my case buttons were used many times,

**first** Is the Menu of the code, since the user has the ability to choose between three choices which are the following:
-Enter its own new coordinates X and Y for the robot 
-Controlling thr robot using the keyboard
-controlling the robot using the keyboard while avoiding obstacles.
So these three options can be choose using collored buttons in a way whenever a button is pressed or chosen from the user it will call a speecific funtion.

**second** going through each choice, if button one has been chosen from the user a unction 'CASE 1' will be called in that case where this function contains two  'widgets.BoundedFloatText' for x and y startin from a specified value 0.5 in my case and can be increased or decreased from the user, 0.1 step per push.
accordingly a new button will appear 'send X and Y', once pressed a new function will called getting the new values of X and Y as inputs and sending them to the robot using the client server.

**Third** if second button of the menu is pushed, other 4 buttons will appear responsible of the guidance of the robot containing the 'up' 'down' 'left' and 'right' buttons which will accordingly call a new function responsible of the velocity of the robot.

**fourth** The third button in the menu will do the same job as the second but a check box will be displayed in a way that will launch the file responsible of the avoidance of the obstacles.

Moreover, two visualisers were included to track the path of the robot and the laserscan.

Third Part:
-----------
As last task a statistical analysis was required, comparing the codes of the first assignment and the assignment of the Professors, where we decided what studies we want to do and which are the things that we want to compare.
In my case, 20 samples have been taken into consideration where i studied the time of each lap, if the robot crashed or not, the wrong directions took from the robot during the path, and in addition i was tracking if any of the silver token were skipped from the robot in the both codes but this study was null for both of the codes.
However, T test and Chi-Square test were applied.

**The T test:**
T-Test, also known as Student’s Test, is based on t-distribution and is considered an appropriate test for comparing two codes in our case.
The aim of this study is to get the t value and compare it with the values in the t table and  accordingly.
This values can be calculating after calculating the standard deviation and then the pooled standard deviation to finally get the estimated error. Comparing the value with the values in the t table first the dof should be known which is the summation of the samples - 2 and if the value compared is greater than 2.01 then the H0 must be rejected which happened in my studies found the excel file.

**chi-square test:**

The chi-square statistic is one way to show a relationship between two categorical variables which was used to compared the crashed and not crashed in my case by First  calculating the expected frequencies on the basis of given hypothesis or on the basis of null hypothesis, then Obtaining the difference between observed and expected frequencies and find out the squares of such differences.After it, Dividing the quantity obtained above by the corresponding expected frequency.And finally sum these values. And also by comparing the values with the chi table, the H0 can be rejected if the error is between 0.1 and 0.2.

**Timing of the codes:**

In this case, the timing of the laps took in both codes where studied to compute all the calculations sited before and to finally get to a conclusion where the hypothesis can be accepted or rejected
in this case the hypothesis are:

H0:The two codes have the same speed

Ha:My robot is faster

since in this case t_calculated > t_table it can reject the H0 and support the Ha

**Wrong directions:**

In this case the direction took by the robot while driving or after crabing and leaving a token behind

H0: both can drive taking a straight direction

Ha: Professor's robot can drive with a straight direction.

and since also in this case t_calculated > t_table this one can reject the H0 and support the Ha

**Crashed/Not crashed**

In this case the Chi-square test is used, in which we are comparing the number of crashes done during the laps and what were the expected values

H0: both code have the same efficiency

Ha: Professor's code is better.

since the result i got is on the left side so the H0 can't be rejected (between 0.1 and 0.2)










Introduction:
-------------
The main purpose of this assignment (#14) is to build a software architecture (Ros 1 in our case) and a simulation environment containing a conveyor belt where 3D boxes are spawned one after the other and detected depending on their colors. Accordingly a robot manipulator (Tiago) is able to estimate their position through an RGB-D camera to grasp the box and put it in its specific bin.
[google] (https://www.google.com)
(https://www.google.com "Google's Homepage")
[I'm an inline-style link with title](https://www.google.com "Google's Homepage")
