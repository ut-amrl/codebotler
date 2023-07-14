---
title: "CodeBotler:"
subtitle: Deploying and Evaluating Language Models for Robot Program Generation
authors: [Francesca Lucchetti, Zichao Hu, Sadanand Modak, Yash Saxena, Anders Freeman, Luisa Mao, Claire Schlesinger, Arjun Guha, Joydeep Biswas]
layout: project
order: 1
---

# Abstract
Recent advancements in the capabilities of large language
models (LLMs) have spurred interest in using them for generating robot task policies from natural language task 
descriptions. Early results indicate that LLMs are indeed capable of generating reasonable general-purpose service robot
programs. However, such initial promise has been tempered
by results showing that 1) LLMs frequently generate imprecise results that are not executable, 
and 2) large-scale testing and evaluation has been hindered by the lack of existing
benchmarks. We contribute **CodeBotler** and **RoboEval** to address the two issues respectively.

# Overview

**CodeBotler** is an open-source tool to generate general-
purpose service robot programs from natural language, and
to deploy such programs on general-purpose autonomous
mobile robots. 

**RoboEval** is a code completion benchmark that test for both correctness and robustness of LLM-generated robot programs. This proposed benchmark 1) evaluate the *execution traces* of programs; 2) checks whether the execution traces satisfy *temporal logic* properties that encode correctness for each task; 3) contains *various the prompts* and
evaluate the resulting programs to evaluate for robustness

<div>
<img src="assets/images/RoboEvalFig1.jpg" style="width:100%"/>
</div>



# Preliminary RoboEval Results
<div>
We have released benchmarks on 5 different tasks:
- **HalloweenList**: Go to every office, and if there is anyone there, ask if they'd like a chocolate, caramel, or gummy. Come back and tell me how many of each we need to buy.
- **LunchBreak**: Ask if Alice and Bob in their offices are up for lunch. If yes, tell them that we'll meet in the lobby in 5 minutes. Come back and tell me who all are joining for lunch.
- **StaplerSupply**: Check every printer room for a stapler, and come back and tell me which ones do not have a stapler.
- **MovieMessenger**: Ask Sally in her office if she wants to go to the cinema with Mark. Go to Mark's office and tell him Sally's answer. If Sally says yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM - then go tell Sally what time Mark is leaving.
- **ElevatorTour**: Go to the elevator. Wait until someone shows up and ask them if they are here for the tour. If yes, welcome them to the university, ask them to follow you, and take them to the main conference room. If not, wait for the next person. When you get to the conference room, say you have arrived at the conference room and also say enjoy your visit here!

<img src="assets/images/results_v3.png" style="width:100%"/>
</div>


# CodeBotler Demo