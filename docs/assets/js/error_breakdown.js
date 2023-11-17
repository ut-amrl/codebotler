const error_ids = ["overall-error", "python-error", "execution-error", "completion-error"];
function show_overall_error(){
    for (let i = 0; i < error_ids.length; i++) {
        id = error_ids[i];
        button = document.getElementById(id);
        button.classList.remove('active');
    }
    button = document.getElementById(error_ids[0]);
    button.classList.toggle('active');
    var trace1 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [85.5, 66.0, 44.8, 45.7, 31.2],
        type: 'bar',
        name: 'Successful Completion',
        marker: {
            color: '#A6B727' 
        }
    };
    var trace2 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [1.1, 11.9, 9.7, 3.4, 3.7],
        type: 'bar',
        name: 'Python Error',
        marker: {
            color: '#FFE680' 
        }
    };
    var trace3 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [1.8, 4.1, 22.0, 30.3, 29.3],
        type: 'bar',
        name: 'Robot Execution Error',
        marker: {
            color: '#FFCC00' 
        }
    };
    var trace4 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [11.7, 18, 23.5, 20.6, 35.8],
        type: 'bar',
        name: 'Task Completion Error',
        marker: {
            color: '#F59E00' 
        }
    };
    var data = [trace1, trace2, trace3, trace4];
    var layout = {
        barmode: 'stack', // Enable stacking
        xaxis: { title: 'Overall Error Breakdown' },
        yaxis: { title: 'Fraction of Total Completions (%)' }
    };
    Plotly.newPlot('error_breakdown', data, layout);
}
function show_python_error(){
    for (let i = 0; i < error_ids.length; i++) {
        id = error_ids[i];
        button = document.getElementById(id);
        button.classList.remove('active');
    }
    button = document.getElementById(error_ids[1]);
    button.classList.toggle('active');
    var trace1 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0, 0, 35.0, 0.7],
        type: 'bar',
        name: 'ValueError',
        marker: {
            color: '#9F3C1C' 
        }
    };
    var trace2 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 3.3, 0, 0, 6.7],
        type: 'bar',
        name: 'UnboundLocalError',
        marker: {
            color: '#F27457' 
        }
    };
    var trace3 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 8.8, 0.3, 18.6, 0],
        type: 'bar',
        name: 'IndexError',
        marker: {
            color: '#15C12C' 
        }
    };
    var trace4 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [100, 84.3, 0.5, 0, 0],
        type: 'bar',
        name: 'NameError',
        marker: {
            color: '#03A696' 
        }
    };
    var trace5 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0, 22.0, 0, 10.1],
        type: 'bar',
        name: 'ZeroDivisionError',
        marker: {
            color: '#9DA464' 
        }
    };
    var trace6 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0.6, 3.4, 0.7, 16.1],
        type: 'bar',
        name: 'SyntaxError',
        marker: {
            color: '#E9E261' 
        }
    };
    var trace7 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0, 0, 0, 0.7],
        type: 'bar',
        name: 'KeyError',
        marker: {
            color: '#F59E00' 
        }
    };
    var trace8 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 2.9, 73.4, 45.7, 0],
        type: 'bar',
        name: 'TypeError',
        marker: {
            color: '#6F8DB9' 
        }
    };
    var trace9 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0, 0.5, 0, 65.8],
        type: 'bar',
        name: 'TimeoutError',
        marker: {
            color: '#253659' 
        }
    };
    var data = [trace1, trace2, trace3, trace4, trace5, trace6, trace7, trace8, trace9];
    var layout = {
        barmode: 'stack', // Enable stacking
        xaxis: { title: 'Python Error Breakdown' },
        yaxis: { title: 'Fraction of Total Python Errors (%)' }
    };
    Plotly.newPlot('error_breakdown', data, layout);
}
function show_execution_error(){
    for (let i = 0; i < error_ids.length; i++) {
        id = error_ids[i];
        button = document.getElementById(id);
        button.classList.remove('active');
    }
    button = document.getElementById(error_ids[2]);
    button.classList.toggle('active');
    var trace1 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0.6, 0.4, 2.0, 5.7],
        type: 'bar',
        name: 'PickInvalidObject',
        marker: {
            color: '#5A5A5A' 
        }
    };
    var trace2 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [92.9, 0, 0, 0, 0],
        type: 'bar',
        name: 'AskInvalidOptions',
        marker: {
            color: '#C0C0C0' 
        }
    };
    var trace3 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [3.6, 10.2, 46.0, 67.3, 59.7],
        type: 'bar',
        name: 'GoToInvalidLocation',
        marker: {
            color: '#FFCC00' 
        }
    };
    var trace4 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [3.5, 64.5, 9.3, 6.7, 6.6],
        type: 'bar',
        name: 'AskNoPerson',
        marker: {
            color: '#9DA464' 
        }
    };
    var trace5 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 23.5, 21.7, 5.9, 20.8],
        type: 'bar',
        name: 'PlaceNoObject',
        marker: {
            color: '#6F8DB9' 
        }
    };
    var trace6 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 1.2, 22.6, 18.1, 7.2],
        type: 'bar',
        name: 'PickWhileHolding',
        marker: {
            color: '#253659' 
        }
    };
    var data = [trace1, trace2, trace3, trace4, trace5, trace6];
    var layout = {
        barmode: 'stack', // Enable stacking
        xaxis: { title: 'Robot Execution Error Breakdown' },
        yaxis: { title: 'Fraction of Total Robot Execution Errors (%)' }
    };
    Plotly.newPlot('error_breakdown', data, layout);
}
function show_completion_error(){
    for (let i = 0; i < error_ids.length; i++) {
        id = error_ids[i];
        button = document.getElementById(id);
        button.classList.remove('active');
    }
    button = document.getElementById(error_ids[3]);
    button.classList.toggle('active');
    var trace1 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0.4, 3.2, 0.7, 2.2],
        type: 'bar',
        name: 'Manipulation at Location',
        marker: {
            color: '#9F3C1C' 
        }
    };
    var trace2 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 1.5, 12.6, 7.7, 9.1],
        type: 'bar',
        name: 'Event Ordering',
        marker: {
            color: '#F24405' 
        }
    };
    var trace3 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [31.7, 22.1, 23.8, 28.7, 33.5],
        type: 'bar',
        name: 'Correct Initial/Terminal',
        marker: {
            color: '#F59E00' 
        }
    };
    var trace4 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0, 0.6, 0.4, 2.0],
        type: 'bar',
        name: 'Location',
        marker: {
            color: '#9DA464' 
        }
    };
    var trace5 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0.5, 6.2, 5.4, 1.4],
        type: 'bar',
        name: 'Exhaustive Search',
        marker: {
            color: '#9EF8EE' 
        }
    };
    var trace6 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0.1, 1.5, 5.9, 4.3, 9.2],
        type: 'bar',
        name: 'Ask Statement at Location',
        marker: {
            color: '#6F8DB9' 
        }
    };
    var trace7 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [68.2, 73.6, 43.1, 51.2, 42.5],
        type: 'bar',
        name: 'Say Statement at Location',
        marker: {
            color: '#04BF9D' 
        }
    };
    var trace8 = {
        x: ['GPT-4', "GPT-3.5", "PaLM2", "CodaLlama", "StarCoder"],
        y: [0, 0.5, 4.6, 1.6, 0],
        type: 'bar',
        name: 'Check Entity Statements at Location',
        marker: {
            color: '#4C2452' 
        }
    };
    var data = [trace1, trace2, trace3, trace4, trace5, trace6, trace7, trace8];
    var layout = {
        barmode: 'stack', // Enable stacking
        xaxis: { title: 'Robot Execution Error Breakdown' },
        yaxis: { title: 'Fraction of Total Robot Execution Errors (%)' }
    };
    Plotly.newPlot('error_breakdown', data, layout);
}
show_overall_error()
