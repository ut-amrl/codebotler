---
title: CodeBotler + RoboEval 
subtitle: Deploying and Evaluating LLMs to Program Service Mobile Robots
authors: [Zichao Hu<sup>1</sup>, Francesca Lucchetti<sup>2</sup>, Claire Schlesinger<sup>2</sup>, Yash Saxena<sup>1</sup>, Anders Freeman<sup>3</sup>, Sadanand Modak<sup>1</sup>, Arjun Guha<sup>2</sup>, Joydeep Biswas<sup>1</sup> ]
layout: project
order: 1
---

<style>
@import url('https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@500&display=swap');
.curly-font {
    font-family: 'Space Grotesk', cursive;
    color: orange;
}
</style>

<div class="text-center">
  <a type="button" class="btn btn-outline-secondary" style="margin:20pt; height:40px;" href="https://github.com/ut-amrl/codebotler">
    <h5>
      <img src="assets/images/github.png" style="height:30px;"/> Code
    </h5>
  </a>

  <a role="button" class="btn btn-outline-secondary" style="margin:20pt; height:40px;" href="https://arxiv.org/abs/2311.11183">
    <h5>
      <img src="assets/images/paper_thumbnail.png" style="height:30px;"/> Paper
    </h5>
  </a>
</div>

<div style="text-align:center">
  <video autoplay muted loop>
    <source src="assets/website_videos/CodeBotler_Animation_Only_compressed.mp4" type="video/mp4"></source>
  </video>
</div>

### <span class="curly-font">CodeBotler</span> is an open-source tool to generate robot programs from natural language using LLMs, and to enable robot-agnostic deployment of such programs. <span class="curly-font">RoboEval</span> is a benchmark to evaluate LLM-generated robot programs for service mobile robots.

<hr>

# Abstract



Recent advancements in large language models (LLMs) have spurred interest in using them for generating robot programs from natural language, with promising initial results.  We investigate the use of LLMs to generate programs for service mobile robots leveraging mobility, perception, and human interaction skills, and where *accurate sequencing and ordering* of actions is crucial for success. We contribute <span class="curly-font">CodeBotler</span>
 , an open-source robot-agnostic tool to program service mobile robots from natural language, and <span class="curly-font">RoboEval</span>, a benchmark for evaluating LLMs’ capabilities of generating programs to complete service robot tasks. <span class="curly-font">CodeBotler</span> performs program generation via few-shot prompting of LLMs with an embedded domain-specific language (eDSL) in Python, and leverages skill abstractions to deploy generated programs on any general-purpose mobile robot. <span class="curly-font">RoboEval</span> evaluates the correctness of generated programs by checking execution traces starting with multiple initial states, and checking whether the traces satisfy temporal logic properties that encode correctness for each task. <span class="curly-font">RoboEval</span> also includes multiple prompts per task to test for the robustness of program generation. We evaluate several popular state-of-the-art LLMs with the <span class="curly-font">RoboEval</span> benchmark, and perform a thorough analysis of the modes of failures, resulting in a taxonomy that highlights common pitfalls of LLMs at generating robot programs. 

<hr>

# <span class="curly-font">CodeBotler</span> Demo
<div id="carouselExampleControls" class="carousel slide" data-ride="carousel" data-interval=10000>
  <div class="carousel-inner">
    <div class="carousel-item active">
      <div class="row">
        <div class="col-md-3"></div>
        <div class="col-md-6 text-center">
          <video autoplay muted loop>
            <source src="assets/website_videos/code_elevatortour.mp4" type="video/mp4"></source>
            <!-- <source src="https://drive.google.com/uc?export=view&id=1Cj_tCNcEckeWKGEWv7esY0S3n7RFRKPQ" type="video/mp4"></source> -->
          </video>
        </div>
        <div class="col-md-3"></div>
      </div>
      <div class="row">
        <div class="col-md-2"></div>
        <div class="col-md-4">
          <video autoplay muted loop>
            <source src="assets/website_videos/elevator_tour/goto_and_wait_compressed.mp4" type="video/mp4"></source>
            <!-- <source src="https://drive.google.com/uc?export=view&id=1kcaZn5TXr1vVgvlSDnO6tkwJ0kJ3JW_6" type="video/mp4"></source> -->
          </video>
          <span class="video-description">1. Go to the elevator</span>
          <hr class="my-hr">
          <video autoplay muted loop >
            <source src="assets/website_videos/elevator_tour/welcome_compressed.mp4" type="video/mp4"></source>
            <!-- <source src="https://drive.google.com/uc?export=view&id=1NuuYdWrhgicB5zAF-1hpDuA8-n3synJT" type="video/mp4"></source> -->
          </video>
          <span class="video-description">3. Take the attendee to the conference room</span>
          <hr class="my-hr">
        </div>
        <div class="col-md-4">
          <video autoplay muted loop>
            <source src="assets/website_videos/elevator_tour/ask_compressed.mp4" type="video/mp4"></source>
            <!-- <source src="https://drive.google.com/uc?export=view&id=13ejDVNvVOz3yDkJKor9XrlB0ZPv18b9z" type="video/mp4"></source> -->
          </video>
          <span class="video-description">2. Ask if they are here for the conference</span>
          <hr class="my-hr">
          <video autoplay muted loop>
            <source src="assets/website_videos/elevator_tour/has_arrived_compressed.mp4" type="video/mp4"></source>
            <!-- <source src="https://drive.google.com/uc?export=view&id=1inW7zaS58trdxUYhmXwTfT6xiJtJC7np" type="video/mp4"></source> -->
          </video>
          <span class="video-description">4. Let the attendee know you have arrived</span>
          <hr class="my-hr">
        </div>
        <div class="col-md-2"></div>
      </div>
    </div>
    <div class="carousel-item">
      <div class="row">
        <div class="col-md-3"></div>
        <div class="col-md-6 text-center">
          <video autoplay muted loop>
            <source src="assets/website_videos/code_findbackpack.mp4" type="video/mp4"></source>
            <!-- <source src="https://drive.google.com/uc?export=view&id=1VEATT18cjUU9zXxs2ySadYiI4Nx1QRXD" type="video/mp4"></source> -->
          </video>
        </div>
        <div class="col-md-3"></div>
      </div>
      <div class="row">
        <div class="col-md-2"></div>
        <div class="col-md-4">
          <video autoplay muted loop>
              <source src="assets/website_videos/findbackpack/goto_compressed.mp4" type="video/mp4"></source>
              <!-- <source src="https://drive.google.com/uc?export=view&id=1u839o3RkhzSVvzn05hs1pmYUmPZ9ovxQ" type="video/mp4"></source> -->
            </video>
            <span class="video-description">1. Go to the art studio</span>
          <hr class="my-hr">
          <video autoplay muted loop>
              <source src="assets/website_videos/findbackpack/find_loc2_compressed.mp4" type="video/mp4"></source>
              <!-- <source src="https://drive.google.com/uc?export=view&id=18qNXgPf8gXxsjG1X_ThFULTc3J87VOlr" type="video/mp4"></source> -->
            </video>
            <span class="video-description">3. Check the sofa area and find the backpack</span>
          <hr class="my-hr">
        </div>
        <div class="col-md-4">
          <video autoplay muted loop>
              <source src="assets/website_videos/findbackpack/find_loc1_compressed.mp4" type="video/mp4"></source>
              <!-- <source src="https://drive.google.com/uc?export=view&id=1hM3DVC64PLIeXdjxk8swHD87nGHl26jb" type="video/mp4"></source> -->
            </video>
            <span class="video-description">2. Check the rest area</span>
          <hr class="my-hr">
          <video autoplay muted loop>
              <source src="assets/website_videos/findbackpack/come_back_compressed.mp4" type="video/mp4"></source>
              <!-- <source src="https://drive.google.com/uc?export=view&id=1cZkaCSXeFtyns5XPjlEHgyQA5GFJ6KPe" type="video/mp4"></source> -->
            </video>
            <span class="video-description">4. Come back and tell where the backpack is</span>
          <hr class="my-hr">
        </div>
        <div class="col-md-2"></div>
      </div>
    </div>
    <div class="carousel-item">
      <div class="row">
        <div class="col-md-3"></div>
        <div class="col-md-6 text-center">
          <video autoplay muted loop>
            <source src="assets/website_videos/code_grilledcheese.mp4" type="video/mp4"></source>
            <!-- <source src="https://drive.google.com/uc?export=view&id=1yuTAAzfKkGm9E5uLsjdM2Wk1ay-4Za5o" type="video/mp4"></source> -->
          </video>
        </div>
        <div class="col-md-3"></div>
      </div>
      <div class="row">
        <div class="col-md-2"></div>
        <div class="col-md-4">
          <video autoplay muted loop>
              <source src="assets/website_videos/grilledcheese/goto_compressed.mp4" type="video/mp4"></source>
              <!-- <source src="https://drive.google.com/uc?export=view&id=1mJwklsuw3fLRy6wAf1PNu1K7XtK0J5NN" type="video/mp4"></source> -->
            </video>
            <span class="video-description">1. Go to Zarko's office</span>
          <hr class="my-hr">
          <video autoplay muted loop>
              <source src="assets/website_videos/grilledcheese/go_back_compressed.mp4" type="video/mp4"></source>
              <!-- <source src="https://drive.google.com/uc?export=view&id=101i2EBUfFl6TnD4UJywFWZlzzNqqVKBn" type="video/mp4"></source> -->
            </video>
            <span class="video-description">3. Come back to the starting location</span>
          <hr class="my-hr">
        </div>
        <div class="col-md-4">
          <video autoplay muted loop>
              <source src="assets/website_videos/grilledcheese/ask_compressed.mp4" type="video/mp4"></source>
              <!-- <source src="https://drive.google.com/uc?export=view&id=18Wwmuelt_aPZzUDRk71IPYJqhkBbBUr-" type="video/mp4"></source> -->
            </video>
            <span class="video-description">2. Ask which ingredients Zarko does not have</span>
          <hr class="my-hr">
          <video autoplay muted loop>
              <source src="assets/website_videos/grilledcheese/say_compressed.mp4" type="video/mp4"></source>
              <!-- <source src="https://drive.google.com/uc?export=view&id=1JSwVuSTkxKrwpBeFpvIouEfxSSDMHVKC" type="video/mp4"></source> -->
            </video>
            <span class="video-description">4. Say what Zarko does not have</span>
          <hr class="my-hr">
        </div>
        <div class="col-md-2"></div>
      </div>
    </div>
  </div>
  <a class="carousel-control-prev" href="#carouselExampleControls" role="button" data-slide="prev">
    <span class="carousel-control-prev-icon" aria-hidden="true"></span>
    <span class="sr-only">Previous</span>
  </a>
  <a class="carousel-control-next" href="#carouselExampleControls" role="button" data-slide="next">
    <span class="carousel-control-next-icon" aria-hidden="true"></span>
    <span class="sr-only">Next</span>
  </a>
</div>

<hr>

# <span class="curly-font">CodeBotler</span> Robot Skills and <span class="curly-font">RoboEval</span> Temporal Checks
<span class="curly-font">CodeBotler</span> leverages an embedded domain-specific language (eDSL) in Python to abstract 8 commonly available service robot skills shown in (a). Given a user task description, <span class="curly-font">CodeBotler</span> constructs a service robot program. <span class="curly-font">RoboEval</span> is a benchmark that contains a suite of 16 different user tasks, each with 5 prompt paraphrases, totalling 80 different prompts. To evaluate a service robot program generated from the benchmark tasks, <span class="curly-font">RoboEval</span> contains a symbolic simulator and a temporal trace evaluator. The symbolic simulator executes the program and outputs a trace of robot skills. Then the temporal trace evaluator uses the <span class="curly-font">RoboEval</span> Temporal Logic (RTL) specifications to evaluate the correctness of the program. The definition of the RTL is shown below in (b).

<div class="text-center">
  <img src="assets/images/Skill_and_RTL.png" />
</div>

<hr>


# <span class="curly-font">RoboEval</span> Benchmark Tasks and Evaluations
We investigate the capabilities and limitations of five popular state-of-the-art LLMs for generating service mobile robot LMPs:

- **[GPT4](https://platform.openai.com/docs/models/gpt-4-and-gpt-4-turbo)** (`gpt-4-0613`): A proprietary LLM from OpenAI capable of general language tasks, including code generation.
- **[GPT3.5](https://platform.openai.com/docs/models/gpt-3-5)** (`text-davinci-003`): A proprietary LLM from OpenAI capable of general language tasks, including code generation.
- **[PaLM 2](https://developers.generativeai.google/models/language)** (`text-bison-001`): A proprietary LLM from Google with multiple capabilities, including code generation.
- **[CodeLlama](https://huggingface.co/codellama/CodeLlama-34b-Python-hf)** (`codellama/CodeLlama-34b-Python-hf`): An open-access version of Llama 2 from Meta specialized on code generations.
- **[StarCoder](https://huggingface.co/bigcode/starcoder)** (`bigcode/starcoder`): An open-source 15.5B parameter LLM trained on 80+ programming languages from [The Stack](https://huggingface.co/datasets/bigcode/the-stack).

These LLMs are evaluated on the <span class="curly-font">RoboEval</span> benchmark which consists of 16 user tasks, each with 5 prompt paraphrases, totaling 80 different prompts. For each prompt, we generate 50 program completions and calculate the pass@1 score. Below, we present the details of each user task and the pass@1 score of each LLM over every prompt.

<div>
  <button type="button" class="btn btn-outline-secondary active" id="CS" onclick="showText('CS')">CountSavory</button>
  <button type="button" class="btn btn-outline-secondary" id="ET" onclick="showText('ET')">Elevator Tour</button>
  <button type="button" class="btn btn-outline-secondary" id="FB" onclick="showText('FB')">Find Backpack</button>
  <button type="button" class="btn btn-outline-secondary" id="GD" onclick="showText('GD')">Get Drink</button>
  <button type="button" class="btn btn-outline-secondary" id="GC" onclick="showText('GC')">Grilled Cheese</button>
  <button type="button" class="btn btn-outline-secondary" id="HL" onclick="showText('HL')">Halloween List</button>
  <button type="button" class="btn btn-outline-secondary" id="HS" onclick="showText('HS')">Halloween Shopping</button>
  <button type="button" class="btn btn-outline-secondary" id="LB" onclick="showText('LB')">Lunch Break</button>
  <button type="button" class="btn btn-outline-secondary" id="LT" onclick="showText('LT')">Lunch Time</button>
  <button type="button" class="btn btn-outline-secondary" id="MD" onclick="showText('MD')">Mail Delivery</button>
  <button type="button" class="btn btn-outline-secondary" id="MM" onclick="showText('MM')">Movie Messenger</button>
  <button type="button" class="btn btn-outline-secondary" id="SG" onclick="showText('SG')">Say Good Day</button>
  <button type="button" class="btn btn-outline-secondary" id="ST" onclick="showText('ST')">Set Temperature</button>
  <button type="button" class="btn btn-outline-secondary" id="SD" onclick="showText('SD')">Stapler Delivery</button>
  <button type="button" class="btn btn-outline-secondary" id="SS" onclick="showText('SS')">Stapler Supply</button>
  <button type="button" class="btn btn-outline-secondary" id="WP" onclick="showText('WP')">Weather Poll</button>
</div>

<table class="table table-hover">
  <thead class="thead-dark">
    <th colspan="4" class="text-center">Task Details</th>
  </thead>
  <tr>
      <th style="width: 10%;">Prompts</th>
      <td colspan="3" style="text-align: left;">
        <div>
          <code id="code1" class="language-plaintext highlighter-rouge"></code>:
          <span id="prompt1"></span>
        </div>
        <div>
          <code id="code2" class="language-plaintext highlighter-rouge"></code>:
          <span id="prompt2"></span>
        </div>
        <div>
          <code id="code3" class="language-plaintext highlighter-rouge"></code>:
          <span id="prompt3"></span>
        </div>
        <div>
          <code id="code4" class="language-plaintext highlighter-rouge"></code>:
          <span id="prompt4"></span>
        </div>
        <div>
          <code id="code5" class="language-plaintext highlighter-rouge"></code>:
          <span id="prompt5"></span>
        </div>
      </td>
  </tr>
  <tr>
      <th style="width: 10%;">Properties</th>
      <td id="property"></td>
      <th style="width: 20%;">Number of World States</th>
      <td id="statenum" style="width: 5%;"></td>
  </tr>

</table>


<div class="row">
  <div class="col-md-6" >
  <div class="video-container">
    <video id="task-video" autoplay muted loop >
      <source src="assets/website_videos/task_table/task_cs.mp4" type="video/mp4"></source>
    </video>
  </div> 
  </div>
  <div class="col-md-6">
    <div id="heatmap"></div>
  </div>
  <div class="col-md-12 text-center">
    <a id="benchmark-url" class="btn btn-dark btn-lg" role="button">
      View RTL Specifications
    </a>
  </div>
</div>
<script src="assets/js/benchmark_tasks.js"></script>
<hr>


## Causes of Program Failures

<div>
  <button type="button" class="btn btn-outline-secondary active" id="overall-error" onclick="show_overall_error()">Overall Error Breakdown</button>
  <button type="button" class="btn btn-outline-secondary" id="python-error" onclick="show_python_error()">Python Error Breakdown</button>
  <button type="button" class="btn btn-outline-secondary" id="execution-error" onclick="show_execution_error()">Robot Execution Error Breakdown</button>
  <button type="button" class="btn btn-outline-secondary" id="completion-error" onclick="show_completion_error()">Task Completion Error Breakdown</button>
</div>

<div id="error_breakdown"></div>
<script src="assets/js/error_breakdown.js"></script>


#### Citation
```shell
@ARTICLE{10416558,
  author={Hu, Zichao and Lucchetti, Francesca and Schlesinger, Claire and Saxena, Yash and Freeman, Anders and Modak, Sadanand and Guha, Arjun and Biswas, Joydeep},
  journal={IEEE Robotics and Automation Letters}, 
  title={Deploying and Evaluating LLMs to Program Service Mobile Robots}, 
  year={2024},
  volume={9},
  number={3},
  pages={2853-2860},
  keywords={Robots;Task analysis;Benchmark testing;Natural languages;Python;Mobile robots;Service robots;Software tools for benchmarking and reproducibility;software tools for robot programming;social HRI;human-centered robotics;service robotics},
  doi={10.1109/LRA.2024.3360020}
}
```