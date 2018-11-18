# SSUGimbal
This is embedded application for 3D Attitude control Robot, using TM4C123G
Using Kalman Filter, control **robot movement**

## Introduction

&nbsp;&nbsp;This is an introduction to the second projects of the **Wireless Communication SYStem Labatory** of *Soongsil University*


## Project Goal

&nbsp;&nbsp;Our main goal is **implementing Kalman Filter for Control 3D gimbal movement** in real HW system.

&nbsp;&nbsp;In order to overcome the limitations of PID control, which is commonly used in control theory, we introduce estimation control.

Following is a video of PID implementation to Model Car. In the final part of racing, our Model car was shaking because of overshooting.

[Model Car Contest](https://serviceapi.nmv.naver.com/flash/convertIframeTag.nhn?vid=5A4841B3B0767AC539C2CE7F8636269FEA2F&outKey=V123b008e8658469e066ec2262953fcca04addb684af4e3d01916c2262953fcca04ad)
> If you want to see video, you have to install site video player, sorry.

 <frame width="544" height="306" src="https://serviceapi.nmv.naver.com/flash/convertIframeTag.nhn?vid=5A4841B3B0767AC539C2CE7F8636269FEA2F&outKey=V123b008e8658469e066ec2262953fcca04addb684af4e3d01916c2262953fcca04ad" frameborder="no" scrolling="no" title="NaverVideo" allow="autoplay; encrypted-media" allowfullscreen></frame>

&nbsp;&nbsp; We implemented the Kalman Filter + control algorithm by C/C++(mainly C).


## Purpose of Project

&nbsp;&nbsp;The reason why we aimed to control the 3D Gimbal is because we thought it was the most efficent way to study controling robot. In futrue, we want to control the natural movement of the robot for helping people.

## Project Environment
- IDE : **[Code Composer Studio]**(http://www.ti.com/tool/ccstudio?DCMP=dsp_ccs_v4&HQS=ccs)
<center> <img src="https://upload.wikimedia.org/wikipedia/en/f/f7/CCS_icon.png"/ width="200" height="200"> </center>
<br>


- BSW Library : **Tivaware**


- Board : **TM4C123G**,(Cortex-M4)
 
<center> <img src="http://processors.wiki.ti.com/images/e/e3/TITivaLaunchpad2A.jpg"/ width="230" height="300"> </center>
<br>

## Project Schedule

Project period : **2017.8.1 ~ 2017.11.10**

> - (1st month) Select the environment(Board, IDE etc.) and the task.
> - (2nd month) Set hardware environment & first PID tunning
> - (2nd month) Kalman Filter tunning using Matlab and Excel calculation
> - (third month) Complete the implementation and make PPT.

DONE.

## Project Statue


- We are completed to implementing Kalman Filter on embedded system
[SNS post of result - See Second post](https://www.instagram.com/p/BbUdTlVj7Ks/)

## Project Team

- JiWon Ashley Ryu [github](https://github.com/AshleyRyu) [Facebook](https://www.facebook.com/JiwonAshleyRyu) 


- JiMin Lee
