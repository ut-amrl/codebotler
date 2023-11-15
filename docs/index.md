---
title: Deploying and Evaluating LLMs to Program Service Mobile Robots
authors: [Zichao Hu<sup>1</sup>, Francesca Lucchetti<sup>2</sup>, Claire Schlesinger<sup>2</sup>, Yash Saxena<sup>1</sup>, Anders Freeman<sup>3</sup>, Sadanand Modak<sup>1</sup>, Arjun Guha<sup>2</sup>, Joydeep Biswas<sup>1</sup> ]
layout: project
order: 1
---

<!-- <div id="carouselExampleControls" class="carousel slide" data-ride="carousel" data-interval="false">
  <div class="carousel-inner">
    <div class="carousel-item active">
      <div class="row">
          <div class="col-md-9">
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1Cj_tCNcEckeWKGEWv7esY0S3n7RFRKPQ" type="video/mp4"></source>
            </video>
          </div>
          <div class="col-md-3">
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1hKHaXNZoxF5bVX8XSayu90UNovSWEXNp" type="video/mp4"></source>
            </video>
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1WeAfAoI6ceC9gLAaA7V_mDndYgpFMUFq" type="video/mp4"></source>
            </video>
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1SRFiJEq6y-wFhCDNNe3A0yEfJeHd_Hj4" type="video/mp4"></source>
            </video>
          </div>
        </div>
    </div>
    <div class="carousel-item">
        <div class="row">
          <div class="col-md-9">
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1VEATT18cjUU9zXxs2ySadYiI4Nx1QRXD" type="video/mp4"></source>
            </video>
          </div>
          <div class="col-md-3">
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=10kWmhERc3PkGKlvOIbhfPJSvnz10prCY" type="video/mp4"></source>
            </video>
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=17CkKLdcsTH0bt5cKukj3-vtAGKIkxDRS" type="video/mp4"></source>
            </video>
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1RwsqgaWHFy8_IeOeRxtmOsctcwW_7ahE" type="video/mp4"></source>
            </video>
          </div>
        </div>
    </div>
    <div class="carousel-item ">
      <div class="row">
          <div class="col-md-9">
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1yuTAAzfKkGm9E5uLsjdM2Wk1ay-4Za5o" type="video/mp4"></source>
            </video>
          </div>
          <div class="col-md-3">
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1MQHng-a7f0uYLjgUCBZbJ7rwyvsRX1aT" type="video/mp4"></source>
            </video>
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1MHGam1Tp3yecic3bGSZNFEtDq6UhheZN" type="video/mp4"></source>
            </video>
            <video autoplay muted loop>
              <source src="https://drive.google.com/uc?export=view&id=1QSTQ7CDOBKerhXyRcUqBMFcHoEEYovj0" type="video/mp4"></source>
            </video>
          </div>
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
</div> -->
<div style="text-align:center">
  <video autoplay muted loop>
    <source src="https://drive.google.com/uc?export=view&id=1QcZ9tN1_fJ-D8bD7VGf4I0aFTSOVXjk3" type="video/mp4"></source>
  </video>
</div>

## Abstract

Recent advancements in large language models (LLMs) have spurred interest in using them for generating robot programs from natural language, with promising initial results.  We investigate the use of LLMs to generate programs for service mobile robots leveraging mobility, perception, and human interaction skills, and where *accurate sequencing and ordering* of actions is crucial for success. We contribute **CodeBotler**, an open-source robot-agnostic tool to program service mobile robots from natural language, and **RoboEval**, a benchmark for evaluating LLMs’ capabilities of generating programs to complete service robot tasks. **CodeBotler** performs program generation via few-shot prompting of LLMs with an embedded domain-specific language (eDSL) in Python, and leverages skill abstractions to deploy generated programs on any general-purpose mobile robot. **RoboEval** evaluates the correctness of generated programs by checking execution traces starting with multiple initial states, and checking whether the traces satisfy temporal logic properties that encode correctness for each task. **RoboEval** also includes multiple prompts per task to test for the robustness of program generation. We evaluate several popular state-of-the-art LLMs with the **RoboEval** benchmark, and perform a thorough analysis of the modes of failures, resulting in a taxonomy that highlights common pitfalls of LLMs at generating robot programs. 




## RoboEval Benchmark Tasks

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
      <th style="width: 10%;">Prompt</th>
      <td colspan="3" style="text-align: left;">
        <div>
          <code id="code1" class="language-plaintext highlighter-rouge">CountSavory-1</code>:
          <span id="prompt1">
            Go to every office, and if there is someone there, ask them whether they'd like a cupcake, ham sandwich, donut, or beef jerky. Come back and tell me how many people chose a savory option.
          </span>
        </div>
        <div>
          <code id="code2" class="language-plaintext highlighter-rouge">CountSavory-2</code>:
          <span id="prompt2">
            Visit all offices. If anyone is present; ask them to choose from the options of cupcake, ham sandwich, donut, or beef jerky. Let me know how many people selected a savory option when you return.
          </span>
        </div>
        <div>
          <code id="code3" class="language-plaintext highlighter-rouge">CountSavory-3</code>:
          <span id="prompt3">
            Go through each office; if someone is there, ask about their preference among cupcake, donut, beef jerky, or ham sandwich. Report the number of individuals who opted for a savory item after returning.
          </span>
        </div>
        <div>
          <code id="code4" class="language-plaintext highlighter-rouge">CountSavory-4</code>:
          <span id="prompt4">
            Go visit each office. If there is a person in the office, inquire about their liking for cupcake, ham sandwich, beef jerky, or donut. Come back and tell me of the count of people who went for a savory option.
          </span>
        </div>
        <div>
          <code id="code5" class="language-plaintext highlighter-rouge">CountSavory-5</code>:
          <span id="prompt5">
            Go to every office, and if there is someone there, ask them whether they'd like a beef jerky, cupcake, ham sandwich, or donut. Come back and tell me how many people preferred a savory option. 
          </span>
        </div>
      </td>
  </tr>
  <tr>
      <th style="width: 10%;">Property</th>
      <td id="property"></td>
      <th style="width: 30%;">Number of World State</th>
      <td style="width: 10%;">4</td>
  </tr>
</table>
<script src="assets/js/benchmark_tasks.js"></script>




## RoboEval Evaluation Results 
We evaluate the correctness and robustness of generate programs for the 16 tasks
in RoboEval using the following LLMs:
- **[GPT4](https://platform.openai.com/docs/models/gpt-4-and-gpt-4-turbo)** (`gpt-4-0613`): A proprietary LLM from OpenAI capable of general language tasks, including code generation.
- **[GPT3.5](https://platform.openai.com/docs/models/gpt-3-5)** (`text-davinci-003`): A proprietary LLM from OpenAI capable of general language tasks, including code generation.
- **[PaLM 2](https://developers.generativeai.google/models/language)** (`text-bison-001`): A proprietary LLM from Google with multiple capabilities, including code generation.
- **[CodeLlama](https://huggingface.co/codellama/CodeLlama-34b-Python-hf)** (`codellama/CodeLlama-34b-Python-hf`): An open-access LLM from Meta capable of general language tasks, including code generation.
- **[StarCoder](https://huggingface.co/bigcode/starcoder)** (`bigcode/starcoder`): An open-source 15.5B parameter LLM trained on 80+ programming languages from [The Stack](https://huggingface.co/datasets/bigcode/the-stack).

We generate 50 program completions for every prompt per task per LLM for evaluation, for a total of 16x5x50=4000 completions per LLM.
We evaluate the pass@1 rates for every prompt, and report the mean pass@1 rate for each task (over all prompt varations per task), along with the min and max pass@1 rates over all prompts for the same task. The results are then aggregated over all tasks by computing the mean, average min, and average max pass@1 rates over all tasks.

<div id="carouselExampleControls2" class="carousel slide" data-ride="carousel">
  <div class="carousel-inner">
    <div class="carousel-item active">
      <img class="d-block w-100" src="https://drive.google.com/uc?export=view&id=1UW0fG9NK6kkfPySN_SeHb8-oa9LOdefr" alt="Second slide">
    </div>
    <div class="carousel-item">
      <img class="d-block w-100" src="https://drive.google.com/uc?export=view&id=1Ul9haGyWYg0HOTo7owdQSpaA5e5T2bFS" alt="Second slide">
    </div>
    <div class="carousel-item">
      <img class="d-block w-100" src="https://drive.google.com/uc?export=view&id=17unjIZ0xGH2OrzkteTgwRppgQ4hsA1kc" alt="Second slide">
    </div>
  </div>
  <a class="carousel-control-prev" href="#carouselExampleControls2" role="button" data-slide="prev">
    <span class="carousel-control-prev-icon" aria-hidden="true"></span>
    <span class="sr-only">Previous</span>
  </a>
  <a class="carousel-control-next" href="#carouselExampleControls2" role="button" data-slide="next">
    <span class="carousel-control-next-icon" aria-hidden="true"></span>
    <span class="sr-only">Next</span>
  </a>
</div>

<!-- <div style="justify-content: center; align-items: center; display: flex;">
<img src="https://drive.google.com/uc?export=view&id=1UW0fG9NK6kkfPySN_SeHb8-oa9LOdefr" style="width:100%; max-width:500px; height:auto;"/>
</div>

We draw the following conclusions from the results:
- None of the programs generated by GPT3.5 and PaLM 2 had any Python errors, and only 25/500 programs (5% completions) generated by StarCoder resulted in run-time errors.
- Over all tasks, GPT3.5 performed the best, with an average pass@1 rate of 64.7%, followed by PaLM 2 (pass@1 rate of 59.7%), and StarCoder (pass@1 rate of 44.6%).
- Varying the prompt for each task resulted in significant changes in pass@1 rates for all LLMs. For example, the pass@1 rate for the `StaplerSupply` task varied from near-0 pass@1 rate to near-100% pass@1 rate for all LLMs, depending on the prompt.

<div style="justify-content: center; align-items: center; display: flex;">
<img src="assets/images/results_v3.png" style="width:100%; max-width:500px; height:auto;"/>
</div>
<div style="justify-content: center; align-items: center; display: flex;">
pass@1 results for PaLM (`text-bison-001`), StarCoder, and GPT3.5 (`text-davinci-003`) - errorbars show variations resulting from rephrasing task prompts.
</div> -->