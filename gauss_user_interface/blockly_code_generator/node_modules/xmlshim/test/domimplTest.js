var xmlshim = require('../');
var xs, dp;

exports.setUp = function(callback) {
    xs = new xmlshim.XMLSerializer();
    dp = new xmlshim.DOMParser();
    callback();
};

exports.testParseSimpleXML = function(test) {
    var doc = dp.parseFromString('<hello-world/>', 'text/xml');
    test.equals(doc.firstChild.nodeName, 'hello-world');
    test.equals(doc.firstChild.namespaceURI, null);
    test.done();
};

exports.testParseSimpleXMLDefaultNS = function(test) {
    var doc = dp.parseFromString('<hello xmlns="http://example.com/schema"><world creator="slartibartfast"/></hello>', 'text/xml');
    test.equals(doc.firstChild.nodeName, 'hello');
    test.equals(doc.firstChild.namespaceURI, 'http://example.com/schema');

    test.equals(doc.firstChild.firstChild.nodeName, 'world');
    test.equals(doc.firstChild.firstChild.namespaceURI, 'http://example.com/schema');

    test.done();
};

exports.testSimpleRoundtrip = function(test) {
    var origText = '<hello-world/>';
    var doc = dp.parseFromString(origText, 'text/xml');
    var serializedText = xs.serializeToString(doc);

    // Remove XML declaration and trim string
    serializedText = serializedText.replace(/^<\?xml[^>]*>/,'');
    serializedText = serializedText.replace(/^\s*/,'');
    serializedText = serializedText.replace(/\s*$/,'');

    test.equals(serializedText, origText);
    test.done();
};

exports['should have access to node type constants through node instances'] = function(test) {
    var doc = dp.parseFromString('<hello-world/>', 'text/xml');
    n = doc.firstChild;
    test.equals(n.ELEMENT_NODE, 1);
    test.done();
};
