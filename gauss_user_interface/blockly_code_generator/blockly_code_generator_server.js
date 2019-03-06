#!/usr/bin/env node

var fs = require('fs');
var net = require('net'); 

var Blockly = require('./gauss_python_generators').Blockly;

Blockly.Python.STATEMENT_PREFIX = 'n.highlight_block(%1)\n';
Blockly.Python.addReservedWords('highlightBlock');

const generateCode = (dirPath) => {
    var filenameRead = dirPath + '/blockly_xml';
    var filenameWrite = dirPath + '/blockly_python';

    try {
        var xmlCode = fs.readFileSync(filenameRead, 'utf8');
    }
    catch (e) {
        return { status: 400, message: 'Could not read file : ' + filenameRead };
    }

    try {
        var xml = Blockly.Xml.textToDom(xmlCode);
    }
    catch (e) {
        return { status: 400, message: 'Could not parse XML from file : ' + filenameRead };
    }

    var workspace = new Blockly.Workspace();
    
    try {
        Blockly.Xml.domToWorkspace(xml, workspace);
    }
    catch (e) {
        return { status: 400, message: 'Failed to parse given Xml' };
    }
    
    var code = '#!/usr/bin/env python\n\nfrom gauss_python_api.gauss_api import *\n'
        + 'import rospy\nrospy.init_node(\'gauss_generated_code_execution\')\nn = Gauss()\n\n';
    
    try {
        code += Blockly.Python.workspaceToCode(workspace);
    }
    catch (e) {
        return { status: 400, message: 'Could not generate code from given Xml' };
    }

    try {
        fs.writeFileSync(filenameWrite, code, 'utf-8');
    }
    catch (e) {
        return { status: 400, message: 'Could not write generated code on file : ' + filenameWrite };
    }
   
    return  { status: 200, message: 'Successfully generated code' };
}

var HOST = '127.0.0.1';
var PORT = '1337';

var server = net.createServer(function (socket) {
    console.log('CONNECTED: ' + socket.remoteAddress + ':' + socket.remotePort);

    socket.on('data', function (dirPath) {
        var response = generateCode(dirPath.toString('utf8'));
        socket.write(JSON.stringify(response));
    });

    socket.on('close', function (data) {
        console.log('Socket connection closed... ');
    });

    socket.on('error', function (error) {
        console.log(error);
    });
}).listen(PORT, HOST);

