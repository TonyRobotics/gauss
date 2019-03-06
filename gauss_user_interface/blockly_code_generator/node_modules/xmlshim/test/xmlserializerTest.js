var xmlshim = require('../');
var xs, doc;

/**
 * Strip xml preambel and leading/trailing whitespace
 */

exports.setUp = function(callback) {
    xs = new xmlshim.XMLSerializer();
    doc = xmlshim.implementation.createDocument('', '', null);
    callback();
};

exports['should write single element non-namespace xml'] = function(test) {
    var expect = '<hello-world/>';
    var result;
    var root = doc.createElementNS(null, 'hello-world');

    doc.appendChild(root);

    result = xs.serializeToString(doc);
    test.equals(expect, result);

    test.done();
};

exports['should write elements with namespace prefix'] = function(test) {
    var expect = '<hello xmlns="http://example.com/"><big:world xmlns:big="http://example.com/big"/></hello>';
    var result;
    var root = doc.createElementNS('http://example.com/', 'hello');
    var child = doc.createElementNS('http://example.com/big', 'big:world');

    root.appendChild(child);
    doc.appendChild(root);

    result = xs.serializeToString(doc);
    test.equals(expect, result);

    test.done();
}

exports['should write non-namespace attributes'] = function(test) {
    var expect = '<hello say="world"/>';
    var result;
    var root = doc.createElementNS(null, 'hello');

    root.setAttribute('say', 'world');
    doc.appendChild(root);

    result = xs.serializeToString(doc);
    test.equals(expect, result);

    test.done();
}

exports['should write attributes with namespace uri'] = function(test) {
    var expect = '<hello aloud:say="world" xmlns:aloud="http://example.com/"/>';
    var result;
    var root = doc.createElementNS(null, 'hello');

    root.setAttributeNS('http://example.com/', 'aloud:say', 'world');
    doc.appendChild(root);

    result = xs.serializeToString(doc);
    test.equals(expect, result);

    test.done();
}

exports['should write text node'] = function(test) {
    var expect = '<hello>world</hello>';
    var result;
    var root = doc.createElementNS(null, 'hello');
    var child = doc.createTextNode('world');

    root.appendChild(child);
    doc.appendChild(root);

    result = xs.serializeToString(doc);
    test.equals(expect, result);

    test.done();
}

//exports['should write CDATA section'] = function(test) {
//    var expect = '<hello><![CDATA[> world <]]></hello>';
//    var result;
//    var root = doc.createElementNS(null, 'hello');
//    var child = doc.createCDATASection('> world <');
//
//    root.appendChild(child);
//    doc.appendChild(root);
//
//    result = xs.serializeToString(doc);
//    test.equals(expect, result);
//
//    test.done();
//}

exports['should write comments'] = function(test) {
    var expect = '<hello><!--world--></hello>';
    var result;
    var root = doc.createElementNS(null, 'hello');
    var child = doc.createComment('world');

    root.appendChild(child);
    doc.appendChild(root);

    result = xs.serializeToString(doc);
    test.equals(expect, result);

    test.done();
}

exports['should write document fragments'] = function(test) {
    var expect = 'hello<world/>';
    var result;
    var frag = doc.createDocumentFragment();

    frag.appendChild(doc.createTextNode('hello'));
    frag.appendChild(doc.createElementNS(null, 'world'));

    result = xs.serializeToString(frag);
    test.equals(expect, result);

    test.done();
}

exports['should write single element node'] = function(test) {
    var expect = '<hello-world/>';
    var result;

    var e = doc.createElementNS(null, 'hello-world');

    result = xs.serializeToString(e);
    test.equals(expect, result);

    test.done();
}

exports['should write single text node'] = function(test) {
    var expect = '&gt;&gt; hello world';
    var result;

    var t = doc.createTextNode('>> hello world');

    result = xs.serializeToString(t);
    test.equals(expect, result);

    test.done();
}
