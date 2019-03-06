var xmlshim = require('../');
var dp;

exports.setUp = function(callback) {
    dp = new xmlshim.DOMParser();
    callback();
};

exports['should parse single element non-namespace xml'] = function(test) {
    var input = '<hello-world/>';
    var doc = dp.parseFromString(input, 'text/xml');
    test.equals(doc.firstChild.nodeName, 'hello-world');
    test.equals(doc.firstChild.namespaceURI, null);
    test.done();
};

exports['should parse elements with default namespace'] = function(test) {
    var input = '<hello xmlns="http://example.com/"><world/></hello>';
    var doc = dp.parseFromString(input, 'text/xml');

    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, 'http://example.com/');

    test.equals(doc.firstChild.firstChild.nodeName, 'world');
    test.equals(doc.firstChild.firstChild.namespaceURI, 'http://example.com/');

    test.done();
};

exports['should parse elements with namespace prefix'] = function(test) {
    var input = '<hello xmlns="http://example.com/"><big:world xmlns:big="http://example.com/big"/></hello>';
    var doc = dp.parseFromString(input, 'text/xml');

    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, 'http://example.com/');

    test.equals(doc.firstChild.firstChild.nodeName, 'big:world');
    test.equals(doc.firstChild.firstChild.namespaceURI, 'http://example.com/big');

    test.done();
}

exports['child elements should inherit namespaces from parent'] = function(test) {
    var input = '<hello xmlns="http://example.com/" xmlns:big="http://example.com/big"><big:world/></hello>';
    var doc = dp.parseFromString(input, 'text/xml');

    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, 'http://example.com/');

    test.equals(doc.firstChild.firstChild.nodeName, 'big:world');
    test.equals(doc.firstChild.firstChild.namespaceURI, 'http://example.com/big');

    test.done();
}

exports['child elements may override namespaces from parent'] = function(test) {
    var input = '<hello xmlns="http://example.com/" xmlns:big="http://example.com/bigwide"><big:world xmlns:big="http://example.com/big"/></hello>';
    var doc = dp.parseFromString(input, 'text/xml');

    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, 'http://example.com/');

    test.equals(doc.firstChild.firstChild.nodeName, 'big:world');
    test.equals(doc.firstChild.firstChild.namespaceURI, 'http://example.com/big');

    test.done();
}

exports['should parse non-namespace attributes'] = function(test) {
    var input = '<hello say="world"/>';
    var doc = dp.parseFromString(input, 'text/xml');

    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, null);
    test.equals(doc.firstChild.getAttributeNode('say').nodeValue, 'world');
    test.equals(doc.firstChild.getAttributeNode('say').namespaceURI, null);

    test.done();
}

exports['should parse attributes with namespace uri'] = function(test) {
    var input = '<hello xmlns:aloud="http://example.com/" aloud:say="world"/>';
    var doc = dp.parseFromString(input, 'text/xml');

    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, null);
    test.equals(doc.firstChild.getAttributeNode('aloud:say').nodeValue, 'world');
    test.equals(doc.firstChild.getAttributeNode('aloud:say').namespaceURI, 'http://example.com/');

    test.done();
}

exports['should parse text node'] = function(test) {
    var input = '<hello>world</hello>';
    var doc = dp.parseFromString(input, 'text/xml');

    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, null);
    test.equals(doc.firstChild.firstChild.nodeType, doc.TEXT_NODE);
    test.equals(doc.firstChild.firstChild.nodeValue, 'world');

    test.done();
}

//exports['should parse CDATA section'] = function(test) {
//    var input = '<hello><![CDATA[> world <]]></hello>';
//    var doc = dp.parseFromString(input, 'text/xml');
//
//    test.equals(doc.firstChild.nodeName, 'hello');
//    test.equals(doc.firstChild.namespaceURI, null);
//    test.equals(doc.firstChild.firstChild.nodeType, doc.CDATA_SECTION_NODE);
//    test.equals(doc.firstChild.firstChild.nodeValue, '> world <');
//
//    test.done();
//}

exports['should parse comments'] = function(test) {
    var input = '<hello><!--world--></hello>';
    var doc = dp.parseFromString(input, 'text/xml');

    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, null);
    test.equals(doc.firstChild.firstChild.nodeType, doc.COMMENT_NODE);
    test.equals(doc.firstChild.firstChild.nodeValue, 'world');

    test.done();
}

exports['should parse text node before empty element into the enclosing element'] = function(test) {
    var input = '<hello>world<br/></hello>';
    var doc = dp.parseFromString(input, 'text/xml');

    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, null);
    test.equals(doc.firstChild.firstChild.nodeType, doc.TEXT_NODE);
    test.equals(doc.firstChild.firstChild.nodeValue, 'world');
    test.equals(doc.firstChild.firstChild.nextSibling.nodeType, doc.ELEMENT_NODE);
    test.equals(doc.firstChild.firstChild.nextSibling.nodeName, 'br');
    test.equals(doc.firstChild.firstChild.nextSibling.namespaceURI, null);

    test.done();
}
