function showText(buttonId) {
    const task_name_ids = ["CS", "ET", "FB", "GD", "GC", "HL", "HS", "LB", "LT", "MD", "MM", "SG", "ST", "SD", "SS", "WP"];
    for (let i = 0; i < task_name_ids.length; i++) {
      id = task_name_ids[i];
      button = document.getElementById(id);
      button.classList.remove('active');
    }
    button = document.getElementById(buttonId);
    button.classList.toggle('active');

    if(buttonId === "CS") {
        prompt1 = "Go to every office, and if there is someone there, ask them whether they'd like a cupcake, ham sandwich, donut, or beef jerky. Come back and tell me how many people chose a savory option.";
        prompt2 = "Visit all offices. If anyone is present; ask them to choose from the options of cupcake, ham sandwich, donut, or beef jerky. Let me know how many people selected a savory option when you return.";
        prompt3 = "Go through each office; if someone is there, ask about their preference among cupcake, donut, beef jerky, or ham sandwich. Report the number of individuals who opted for a savory item after returning.";
        prompt4 = "Go visit each office. If there is a person in the office, inquire about their liking for cupcake, ham sandwich, beef jerky, or donut. Come back and tell me of the count of people who went for a savory option.";
        prompt5 = "Go to every office, and if there is someone there, ask them whether they'd like a beef jerky, cupcake, ham sandwich, or donut. Come back and tell me how many people preferred a savory option."
        code = "CountSavory";
        property = "Navigation, Perception, Ask Human, Loops, Arithmetic, Commonsense Reasoning";
        statenum = 4;
    } else if(buttonId === "ET") {
        prompt1 = "Go to the elevator. Wait until someone shows up and ask them if they are here for the tour. If yes, welcome them to the university, tell them to follow you, and take them to the main conference room. If not, wait for the next person. When you get to the conference room, say you have arrived at the conference room and also say enjoy your visit here!"
        prompt2 = "Go to the elevator, and wait until someone shows up. Ask them if they are here for the tour. If yes, welcome them to the university, tell them to follow you, and take them to the main conference room. If not, wait for the next person. When you get to the conference room, say you have arrived at the conference room and wish them an enjoyable visit."
        prompt3 = "Go to the elevator and wait for someone to arrive. Inquire if they are here for the tour, and if they say yes, welcome them to the university and tell them to follow you. Then take them to the main conference room. If they say no, wait for someone else. When you get to the conference room, announce your arrival at the conference room and wish them an enjoyable visit."
        prompt4 = "Go to the elevator and wait for a person to show up. When someone shows up, ask them if they are here for the tour, and if they say yes, welcome them to the university and tell them to follow you. Then take them to the main conference room. If they say no, wait for the next person. When you get to the conference room, announce your arrival at the conference room and wish them an enjoyable visit."
        prompt5 = "Go to the elevator, wait until someone arrives, and then ask them if they are here for the tour. If they say yes, welcome them to the university and tell them to follow you, and guide them to the main conference room. If they say no, wait for the next person. When you get to the conference room, announce your arrival at the conference room and wish them an enjoyable visit."
        code = "ElevatorTour";
        property = "Navigation, Perception, Ask Human, Loops";
        statenum = 3;
    } else if(buttonId === "FB") {
        prompt1 = "Check where I left my backpack in all of the conference rooms and bring it back to me.";
        prompt2 = "Search all the conference rooms for my backpack and return it to me.";
        prompt3 = "Please find my backpack in each conference room and then bring it to me.";
        prompt4 = "Go through every conference room and find my backpack, then bring it back to me.";
        prompt5 = "Look in all of the conference rooms to search for my backpack and then bring it back to me.";
        code = "FindBackpack";
        property = "Navigation, Perception, Manipulation, Loops";
        statenum = 3;
    } else if(buttonId === "GD") {
        prompt1 = "Go to the lobby, and ask if the visitor would like a bottle of water, iced tea, or lemonade. Bring what they ask for from the kitchen.";
        prompt2 = "Ask any visitor in the lobby if they would like a bottle of water, iced tea, or lemonade. Bring what they ask for from the kitchen.";
        prompt3 = "Go to the lobby. Ask if the visitor wants: a bottle of water, iced tea, or lemonade. Go to the kitchen and bring the visitor back what they asked for.";
        prompt4 = "Go to the lobby and ask any person there if they want iced tea, lemonade or a bottle of water. Bring whatever they asked for back from the kitchen.";
        prompt5 = "Go to the lobby, and ask if the visitor would like a bottle of water, iced tea, or lemonade. If they want a bottle of water, bring it to them from the kitchen. If they want iced tea, bring it to them from the kitchen. If they want lemonade, bring it to them from the kitchen.";
        code = "GetDrink";
        property = "Navigation, Perception, Ask Human, Manipulation";
        statenum = 3;
    } else if(buttonId === "GC") {
        prompt1 = "Compile a list of ingredients for grilled cheese. Go to Zarko’s office, and ask him which of them he has. Come back and tell me what he does not have.";
        prompt2 = "Create a list of ingredients needed to make grilled cheese. Visit Zarko's office and inquire which of them he possesses. Return and inform me of the items he does not have.";
        prompt3 = "Put together a compilation of grilled cheese ingredients. Head to Zarko's office, inquire about his available ingredients, and then update me on what he does not have.";
        prompt4 = "Compile the necessary ingredients for grilled cheese. Go to Zarko's office, ask him about his available items, and then report back to me which items he does not have.";
        prompt5 = "Put together a list of grilled cheese ingredients. Head to Zarko's office and inquire about his available ingredients. Return and inform me of what he is missing.";
        code = "GrilledCheese";
        property = "Navigation, Perception, Ask Human, Manipulation, Commonsense Reasoning";
        statenum = 1;
    } else if(buttonId === "HL") {
        prompt1 = "Go to every office, and if there is anyone there, ask if they'd like a chocolate, caramel, or gummy. Come back and tell me how many of each we need to buy.";
        prompt2 = "Go to every office, and if there is a person there, ask them if they'd like a chocolate, caramel, or gummy. Come back and tell me how many of each we need to buy.";
        prompt3 = "Check with every occupied office to see if the occupant would like a chocolate, caramel, or gummy. Let me know how many of each we need to buy.";
        prompt4 = "Find every occupied office and ask their occupants whether they would like a chocolate, caramel, or gummy. Let me know how many of each we need to buy.";
        prompt5 = "Go to every office with a person, and ask them if they would like a chocolate, caramel, or gummy, then come back and tell me how many of each people asked for.";
        code = "HalloweenList";
        property = "Navigation, Perception, Ask Human, Loops, Arithmetic"
        statenum = 3;
    } else if(buttonId === "HS") {
        prompt1 = "Go to every office, and if there is anyone there, ask them how many from 0 to 4 of the following treats they’d like: chocolate, caramel, and gummy. Come back and tell me how many of each we need to buy.";
        prompt2 = "For each office, check if there is anyone there. If there is, ask the person to choose how many (0-4) of each of the following they'd like: caramel, chocolate, or gummy. Come back and tell me how many of each we need to buy.";
        prompt3 = "Visit every office. If there is a person in the office, ask the person to select 0 to 4 for each of the following categories: gummy, chocolate, or caramel. Return to me and say how many of each we need to buy.";
        prompt4 = "Go to each office. If no person is in the office, skip this office. Otherwise, ask the person in the office to whether they'd like 0,1,2,3, or 4 for each of the following: gummy, caramel, or chocolate. Return to me and say how many of each we need to buy.";
        prompt5 = "Go to every office and skip any office without a person in there. Otherwise, ask the person in the office to decide on how many, from zero to four, of the following they’d like: gummy, caramel, or chocolate. Return to me and say how many of each we need to buy.";
        code = "HalloweenShopping";
        property = "Navigation, Perception, Ask Human, Loops, Arithmetic";
        statenum = 5;
    } else if(buttonId === "LB") {
        prompt1 = "Ask if Alice and Bob in their offices are up for lunch. If yes, tell them that we'll meet in the lobby in 5 minutes. Come back and tell me who all are joining for lunch.";
        prompt2 = "Ask Alice in her office if she is up for lunch, and if yes, tell her that we will meet in the lobby in 5 minutes. Do the same for Bob. Come back and tell me who all are joining for lunch.";
        prompt3 = "Go to Alice’s and Bob’s offices and ask each of them if they are up for lunch. If they say yes, tell them that we will meet in the lobby in 5 minutes. Come back and tell me who all are joining for lunch.";
        prompt4 = "Ask Alice and Bob if they are up for lunch. If they say yes, tell them that we will meet in the lobby in five minutes. Come back and tell me who all are joining for lunch.";
        prompt5 = "Go to Alice's office and ask her if she is free for lunch. If she says yes, tell her that we will meet in the lobby in five minutes. Do the same for Bob. Come back and tell me who will join for lunch.";
        code = "LunchBreak";
        property = "Navigation, Perception, Ask Human";
        statenum = 4;
    } else if(buttonId === "LT") {
        prompt1 = "Go to Jill's office and ask her if she'd like to go for lunch tomorrow. If yes, ask her at what time (offer her some reasonable options). Come back and tell me what she said.";
        prompt2 = "Go to Jill’s office and ask her whether she's free to go for lunch tomorrow. If yes, ask her when (give her some reasonable times), and come back and tell me what she said.";
        prompt3 = "Visit Jill’s office and ask her if she would like to go for lunch tomorrow. If yes, ask her when (offer some reasonable lunch time options). Let me know what she said.";
        prompt4 = "Ask Jill in her office whether she will be free to go for lunch tomorrow. If yes, offer her some reasonable time options to choose from, and come back and tell me what she said.";
        prompt5 = "Ask Jill in her office if she want to go for lunch tomorrow. If yes, give her some reasonable time options and ask her to choose a time. Finally come back and tell me what she said.";
        code = "LunchTime";
        property = "Navigation, Perception, Ask Human, Commonsense Reasoning";
        statenum = 2;
    } else if(buttonId === "MD") {
        prompt1 = "Pick up a mail bin from the mail room. Go to every office and ask if they have any mail they'd like delivered. If so, ask them to put it in the mail bin. Finally, meet me in the mail room with all the mail. Place the mail bin back and tell me which offices they are from.";
        prompt2 = "Go to the mail room to pick a mail bin. Ask at every office if they have any mail to deliver. If they reply yes, ask them to put the mail in the bin. Return to the mail room with the mail. Tell me at the mail room which offices these mail are from and put the mail bin back.";
        prompt3 = "Grab a mail bin from the mail room. Go to every office, and ask if they have any mail to deliver. Ask them to put all mail in the bin if there is any. Put the mail bin back in the mail room and report which offices the mail is from.";
        prompt4 = "Find a mail bin in the mail room and carry it with you. Collect mail from every office by asking if there is any mail they’d like delivered. If yes, ask them to put it in the bin. Return the bin to the mail room and say which offices sent mail.";
        prompt5 = "Take a mail bin from the mail room. For every office, ask if there is any mail to deliver and if someone says yes, ask them to put all mail in the bin. Finally find me at the mail room. Put the mail bin away, and tell me which offices had mail.";
        code = "MailDelivery";
        property = "Navigation, Perception, Manipulation, Ask Human, Loops";
        statenum = 4;
    } else if(buttonId === "MM") {
        prompt1 = "Ask Sally in her office if she wants to go to the cinema with Mark. Go to Mark's office and tell him Sally’s answer. If Sally says yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM - then go tell Sally what time Mark is leaving.";
        prompt2 = "Ask Sally if she wants to go to the cinema with Mark. Tell Mark what Sally said, and if Sally said yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM, and then go tell Sally what time Mark is leaving.";
        prompt3 = "Ask Sally if she wants to go to the cinema with Mark. Tell Mark what Sally said, and if yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM. If Sally said yes, go tell Sally what time Mark is leaving.";
        prompt4 = "Go to Sally's office and ask her if she wants to go to the cinema with Mark. Tell Mark what Sally said, and if yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM, and then go back to Sally's office and tell her what Mark said.";
        prompt5 = "Ask Sally if she wants to go to the cinema with Mark. Tell Mark what Sally said, and if yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM. Then then go tell Sally what Mark said.";
        code = "MovieMessenger";
        property = "Navigation, Perception, Ask Human";
        statenum = 3;
    } else if(buttonId === "SG") {
        prompt1 = "Go to every lab and if there is a person there, say Good Day!";
        prompt2 = "Visit each lab and if someone is present, greet them with a pleasant Good Day!";
        prompt3 = "Go through all the labs, and if there's a person inside, say “Good Day!” to them.";
        prompt4 = "Head to each lab and in case there's a person present, bid them a Good Day!";
        prompt5 = "Visit every lab space and when encountering an individual, tell them a Good Day!";
        code = "SayGoodDay";
        property = "Navigation, Perception, Ask Human, Loops";
        statenum = 4;
    } else if(buttonId === "ST") {
        prompt1 = "The thermostat is set to 72 degrees. Go to Arjun’s office and ask him if he’d like it to be warmer or colder. Come back and tell me what temperature I should set it to.";
        prompt2 = "Ask Arjun in his office if he would like it to be warmer or colder. The thermostat is currently set to 72 degrees. Come back and tell me what temperature I should set the thermostat to based on what Arjun says.";
        prompt3 = "The thermostat is currently set to 72 degrees. Go ask if Arjun thinks his office should be colder or warmer. Come back and tell me what temperature in degrees I should change the thermostat to.";
        prompt4 = "Go to Arjun’s office and ask him if he would like his office to be warmer or colder. The thermostat says the temperature in the office is currently 72 degrees. Come back and tell me what temperature I should reset it to based on Arjun’s preferences.";
        prompt5 = "The thermostat is set to 72 degrees. Go to Arjun’s office and ask if he would like the temperature to be set to a warmer or colder value. Return and tell me what temperature in degrees I should set the thermostat to. ";
        code = "SetTemperature";
        property = "Navigation, Perception, Ask Human, Commonsense Reasoning";
        statenum = 2;
    } else if(buttonId === "SD") {
        prompt1 = "Check every printer room for a stapler, and deliver a stapler from the supply room to every printer room without a stapler.";
        prompt2 = "Go to every printer room and check if there is a stapler. For each visited room that does not have a stapler, deliver a stapler from the supply room to that room.";
        prompt3 = "Check if there is a stapler in every printer room. For each visited room without a stapler, deliver a stapler from the supply room to that room.";
        prompt4 = "Make sure there is a stapler in each printer room. Otherwise, bring a stapler from the supply room to each printer room that currently lacks one.";
        prompt5 = "Check if every printer room has a stapler. If not, put a stapler from the supply room to every printer room without a stapler.";
        code = "StaplerDelivery";
        property = "Navigation, Perception, Manipulation, Loops";
        statenum = 3;
    } else if(buttonId === "SS") {
        prompt1 = "Check every printer room for a stapler, and come back and tell me which ones do not have a stapler.";
        prompt2 = "Go to every printer room and check to see if there is a stapler. Come back and tell me which printer rooms do not have a stapler.";
        prompt3 = "Go find all printer rooms that do not have a stapler. Come back and tell me.";
        prompt4 = "Check all printer rooms for staplers, and come back and tell me which ones do not have any staplers.";
        prompt5 = "Tell me the name of every printer room that does not have a stapler.";
        code = "StaplerSuppy";
        property = "Navigation, Perception, Loops";
        statenum = 3;
    } else if(buttonId === "WP") {
        prompt1 = "Go to the main entrance, and wait for someone to show up. Ask them if they feel it is hot out. Repeat this until you have 10 responses, and then come back and tell me what percent of people think it is hot out.";
        prompt2 = "Go to the main entrance. When someone shows up, ask them if they feel that it’s hot out. Do this until you collect ten people’s responses. Come back and tell me which percent of people think it’s hot out.";
        prompt3 = "Go wait for someone at the main entrance. Ask them if they feel it is hot out, then wait for the next person and ask the same question. Do this until you have ten responses, then come back and report the percentage of people who think it is hot out.";
        prompt4 = "Go to the main entrance. If there is no one there, wait for someone. Ask them if they feel it is hot out. Ask 10 people this same question until you have 10 replies. Finally, come back and tell me the percentage of people that think it’s hot outside.";
        prompt5 = "Go to the main entrance, and if there is no one there, wait for someone to show up. Ask them if they feel it is hot out. Wait for the next person and ask the same question again. Do this 10 times. Come back and report the percentage of people who think it’s hot out.";
        code = "WeatherPoll";
        property = "Navigation, Perception, Ask Human, Loops, Arithmetic";
        statenum = 5;
    } 

    document.getElementById("code1").innerHTML = code + "-1";
    document.getElementById("code2").innerHTML = code + "-2";
    document.getElementById("code3").innerHTML = code + "-3";
    document.getElementById("code4").innerHTML = code + "-4";
    document.getElementById("code5").innerHTML = code + "-5";

    document.getElementById("prompt1").innerHTML = prompt1;
    document.getElementById("prompt2").innerHTML = prompt2;
    document.getElementById("prompt3").innerHTML = prompt3;
    document.getElementById("prompt4").innerHTML = prompt4;
    document.getElementById("prompt5").innerHTML = prompt5;

    document.getElementById("property").innerHTML = property;
    document.getElementById("statenum").innerHTML = statenum;
    
    handle_plot(buttonId);
    handle_video(buttonId);

    url = "https://github.com/ut-amrl/codebotler-dev/blob/main/benchmark/tasks/CountSavory.py";
    document.getElementById("benchmark-url").href = url;
}

showText("CS")

// video
function handle_video(buttonId){
    
    if(buttonId === "CS") {
        src = "assets/website_videos/task_table/task_cs.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1UI5geAxiFlY7tftUe_xVEHiPwkcsALon";
    } else if(buttonId === "ET") {
        src = "assets/website_videos/task_table/task_et.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1SlZTnc0sXgYDh1WusK_vTOcGT9H-6zHC";
    } else if(buttonId === "FB") {
        src = "assets/website_videos/task_table/task_fb.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1OaIny7X8YRE4f8fK3jFoV2jpQLVwzF-U";
    } else if(buttonId === "GD") {
        src = "assets/website_videos/task_table/task_gd.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1IODJC2tCsCact_LRNo2-w5gOSARRB-Hk";
    } else if(buttonId === "GC") {
        src = "assets/website_videos/task_table/task_gc.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1efJzSWL-WMK1EsVQ2KUEUPmwoK4cNNng";
    } else if(buttonId === "HL") {
        src = "assets/website_videos/task_table/task_hl.mp4"
        // src = "https://drive.google.com/uc?export=view&id=10b6AXg_sdW9uQJXSKzvtMxnVvoJyfk-L";
    } else if(buttonId === "HS") {
        src = "assets/website_videos/task_table/task_hs.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1WjZAwQzGqPL70fO2eiW4vpULM6_ilLIF";
    } else if(buttonId === "LB") {
        src = "assets/website_videos/task_table/task_lb.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1x6opuyMJo5SzA0vf0Mf354HFxCTbFWfr";
    } else if(buttonId === "LT") {
        src = "assets/website_videos/task_table/task_lt.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1ywu4-BZxdPYQ2sARJtNX8SyPQWg70EqO";
    } else if(buttonId === "MD") {
        src = "assets/website_videos/task_table/task_md.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1EZnexZBLDfW5ODGyfly_8n7V7t93B0vn";
    } else if(buttonId === "MM") {
        src = "assets/website_videos/task_table/task_mm.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1BLqCN2oa2QXnOGV3x1imP0bYGHSMtKQP";
    } else if(buttonId === "SG") {
        src = "assets/website_videos/task_table/task_sg.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1aliA3CMHdHF2dQMG1w-HupNwBuWXfEdu";
    } else if(buttonId === "ST") {
        src = "assets/website_videos/task_table/task_st.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1W9E_AHQ7_4I433wJHGDYWRwZSsvi9nm-";
    } else if(buttonId === "SD") {
    src = "assets/website_videos/task_table/task_sd.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1sKql9hTIVsGOiMjSDqgatwVJLpoJVKp-";
    } else if(buttonId === "SS") {
        src = "assets/website_videos/task_table/task_ss.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1QSpM7q7cVyxZI5DCalk-ZN65TNARxDsq";
    } else if(buttonId === "WP") {
        src = "assets/website_videos/task_table/task_wp.mp4"
        // src = "https://drive.google.com/uc?export=view&id=1YhhUiYpTT0suXTg6_LjNcmZfYgcpWbq0";

    }

    var videoPlayer = document.getElementById('task-video');
    videoPlayer.src = src;
    videoPlayer.load();
    videoPlayer.play();
}

// plot heatmap
function handle_plot(buttonId){
    if(buttonId === "CS") {
        zData = [[1.,    1.   , 1. ,   0.38  ,0.   ],
                 [1. ,   1.   , 1.  ,  0.1   ,0.   ],
                 [0.975 ,1.   , 1.   , 0.64  ,0.   ],
                 [1. ,   0.48,  0.92  ,0.   , 0.   ],
                 [1.  ,  1.,    1.    ,0.66,  0.   ]];
    } else if(buttonId === "ET") {
        zData = [[1.0, 1.0, 1.0, 1.0, 0.12], [1.0, 1.0, 1.0, 0.96, 0.0], [1.0, 1.0, 1.0, 1.0, 0.0], [1.0, 1.0, 1.0, 1.0, 0.06], [1.0, 1.0, 1.0, 1.0, 0.0]];
    } else if(buttonId === "FB") {
        zData = [[1.0, 1.0, 0.14, 1.0, 0.0], [1.0, 0.92, 0.0, 1.0, 0.06], [1.0, 1.0, 0.0, 0.78, 0.0], [1.0, 0.98, 0.34, 1.0, 0.0], [1.0, 1.0, 0.06, 1.0, 0.0]];
    } else if(buttonId === "GD") {
        zData = [[1.0, 1.0, 0.52, 0.68, 1.0], [1.0, 1.0, 0.92, 0.94, 0.36], [1.0, 1.0, 0.74, 0.26, 0.86], [1.0, 1.0, 1.0, 0.96, 0.92], [1.0, 0.56, 1.0, 0.92, 1.0]];
    } else if(buttonId === "GC") {
        zData = [[1.0, 0.0, 0.0, 0.74, 0.0], [0.0, 0.0, 0.08, 0.14, 0.06], [0.075, 0.0, 0.0, 0.0, 0.04], [0.975, 0.54, 0.0, 0.06, 0.0], [0.275, 0.58, 0.0, 0.04, 0.24]];
    } else if(buttonId === "HL") {
        zData = [[1.0, 0.96, 0.96, 1.0, 0.32], [1.0, 0.9, 0.94, 1.0, 0.52], [1.0, 0.8, 0.02, 0.2, 0.48], [0.95, 0.9, 0.76, 0.84, 0.22], [1.0, 0.98, 0.94, 0.94, 0.74]];
    } else if(buttonId === "HS") {
        zData = [[0.975, 0.82, 0.06, 1.0, 0.0], [0.975, 0.58, 0.22, 0.0, 0.0], [1.0, 1.0, 0.0, 0.98, 0.0], [1.0, 0.92, 0.18, 1.0, 0.0], [0.95, 0.66, 0.16, 0.34, 0.0]];
    } else if(buttonId === "LB") {
        zData = [[1.0, 0.96, 0.0, 0.0, 0.0], [1.0, 1.0, 0.0, 0.0, 0.0], [1.0, 0.14, 0.0, 0.0, 0.0], [0.85, 0.12, 0.0, 0.0, 0.0], [1.0, 0.7, 0.0, 0.0, 0.0]];
    } else if(buttonId === "LT") {
        zData = [[1.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0], [0.05, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0]];
    } else if(buttonId === "MD") {
        zData = [[0.525, 0.46, 0.0, 0.0, 0.0], [1.0, 1.0, 0.0, 0.0, 0.0], [0.025, 0.0, 0.0, 0.0, 0.0], [0.325, 0.06, 0.0, 0.0, 0.0], [0.2, 0.0, 0.0, 0.0, 0.04]];
    } else if(buttonId === "MM") {
        zData = [[1.0, 1.0, 1.0, 0.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 0.0, 0.66], [1.0, 1.0, 0.98, 1.0, 1.0], [1.0, 1.0, 0.82, 0.0, 0.88]];
    } else if(buttonId === "SG") {
        zData = [[1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 0.98, 1.0, 1.0, 1.0], [0.875, 1.0, 1.0, 0.54, 1.0], [1.0, 0.96, 1.0, 1.0, 1.0], [1.0, 1.0, 0.98, 1.0, 1.0]];
    } else if(buttonId === "ST") {
        zData = [[0.1, 0.8, 0.0, 0.0, 0.26], [0.325, 0.0, 0.0, 0.0, 0.0], [0.6, 0.06, 0.0, 0.0, 0.0], [1.0, 0.06, 0.0, 0.0, 0.14], [0.65, 0.0, 0.0, 0.0, 0.08]];
    } else if(buttonId === "SD") {
        zData = [[1.0, 1.0, 1.0, 0.9, 0.22], [1.0, 1.0, 1.0, 1.0, 0.78], [1.0, 1.0, 1.0, 1.0, 0.52], [1.0, 1.0, 0.0, 1.0, 1.0], [1.0, 0.9, 0.9, 0.88, 0.4]];
    } else if(buttonId === "SS") {
        zData = [[1.0, 1.0, 1.0, 0.34, 1.0], [1.0, 1.0, 0.98, 0.44, 1.0], [1.0, 1.0, 0.12, 0.8, 1.0], [1.0, 1.0, 1.0, 0.38, 1.0], [0.025, 0.0, 0.0, 0.0, 1.0]];
    } else if(buttonId === "WP") {
        zData = [[0.6, 0.0, 0.0, 0.1, 0.0], [0.975, 0.0, 0.02, 0.04, 0.0], [0.875, 0.0, 0.0, 0.08, 0.02], [0.85, 0.0, 0.0, 0.0, 0.0], [0.375, 0.0, 0.08, 0.0, 0.0]];
    }

    plot_heatmap(zData);
}



// Function to determine text color based on cell value
function getTextColor(value) {
    return value < 0.5 ? 'white' : 'black';
}

function getFontSize(width) {
    // Simple formula to calculate font size based on width
    // This is a simple linear scaling. You can replace this with any formula you like
    return Math.max(5, width * 0.008); // Minimum font size of 10px
}
// Use this function to update the layout of the chart when the window resizes
function updateLayout() {
    console.log(getFontSize(window.innerWidth));
    console.log(window.innerWidth)
    var update = {
      'layout.font.size': getFontSize(window.innerWidth)
      // You can also add updates for titlefont, xaxis.titlefont, yaxis.titlefont, etc.
    };
  
    Plotly.relayout('heatmap', update);
}
window.onresize = updateLayout;


function plot_heatmap(zData){
    // Generate text color for each cell
    var textColor = zData.map(row => row.map(getTextColor));

    // Prepare text data
    var textData = zData.map(row => row.map(value => value.toFixed(2)));

    var heatmapTrace = {
        z: zData,
        type: 'heatmap',
        colorscale: 'Viridis',
        showscale: true,
        hoverinfo: 'skip',
    };

    // Layout configuration
    var layout = {
        title: "pass@1 score per prompt per LLM",
        xaxis: {
            tickmode: 'array',
            tickvals: [0, 1, 2, 3, 4],
            ticktext: ['GPT-4', 'GPT-3.5', 'PaLM2', 'CodeLlama', 'StarCoder']
        },
        yaxis: {
            title: 'Prompt ID',
            tickmode: 'array',
            tickvals: [0, 1, 2, 3, 4],
            ticktext: ['1', '2', '3', '4', '5']
        },
        height: 300,
        autosize: true,
        font: {
            size: getFontSize(window.innerWidth) // Set initial font size
        },
        margin: {
            t: 50, // Top margin
            r: 10, // Right margin
            b: 50, // Bottom margin
            l: 50  // Left margin
        }
    };
    var config = {
        responsive: true,
        displayModeBar: false // This will hide the mode bar
    };    

    // Manually set text colors based on calculated contrast
    var data = [heatmapTrace];
        for (var i = 0; i < textData.length; i++) {
        for (var j = 0; j < textData[i].length; j++) {
            data.push({
            x: [j],
            y: [i],
            text: [textData[i][j]],
            mode: 'text',
            type: 'scatter',
            textfont: {
                color: textColor[i][j],
                size: getFontSize(window.innerWidth)
            },
            showlegend: false,
            hoverinfo: 'none',
            hovertemplate: 'pass@1: %{text}<extra></extra>' // No trace name will be displayed
            });
        }
    }

    // Creating the plot
    Plotly.newPlot('heatmap', data, layout, config);
}