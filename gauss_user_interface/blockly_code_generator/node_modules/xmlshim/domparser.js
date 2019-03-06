/**
 * Constructs a function which installs SAX handlers into a libxmljs.SaxParser
 * turning it into a DOM parser for the given doc.
 *
 * The function returned by this function will typically be used as the
 * argument of lixmljs.SaxParser Constructor.
 * <code>
 * var dom = require("jsdom").level(3, 'core'),
 *     xml = require("libxmljs"),
 *     domparser = require("./domparser.js");
 *
 * var doc = new dom.Document();
 * var parser = new xml.SaxParser(domparser.handlersForDocument(doc));
 * parser.parseFile('example.xml');
 * // doc is now populated with the contents of example.xml file
 * </code>
 *
 * @param   doc A document instance implementing at least DOM Core Level 2.
 * @returns An object suitable for passing to the libxmljs.SaxParser and
 *          libixmljs.PushParser constructors.
 */
exports.handlersForDocument = function(doc) {
    /** @private */
    var currentElement = doc;
    /** @private */
    var currentCharacters = '';
    /** @private */
    var currentCdata = '';

    /**
     * Add text node or CDATA section before starting or ending an element
     */
    function flushData() {
        if (currentCharacters) {
            currentElement.appendChild(
                    doc.createTextNode(currentCharacters));
            currentCharacters = '';
        }
        else if (currentCdata) {
            currentElement.appendChild(
                    doc.createCDATASection(currentCdata));
            currentCdata = '';
        }
    }

    return {

        startDocument: function() {},

        endDocument: function() {},

        startElementNS: function(elem, attrs, prefix, uri, namespaces) {
            var element;

            // Finish preceeding text node or CDATA section if any.
            flushData();

            // Create element
            if (uri) {
                if (prefix) {
                    element = doc.createElementNS(uri, prefix + ':' + elem);
                }
                else {
                    element = doc.createElementNS(uri, elem);
                }
            }
            else {
                element = doc.createElementNS(null, elem);
            }

            // Add attributes
            attrs.forEach(function(inattr) {
                var attrLocalName = inattr[0];
                var attrPrefix = inattr[1];
                var attrUri = inattr[2];
                var attrValue = inattr[3];

                if (attrPrefix) {
                    element.setAttributeNS(attrUri,
                        attrPrefix + ':' + attrLocalName, attrValue);
                }
                else {
                    element.setAttribute(attrLocalName, attrValue);
                }
            });

            // Add namespace attributes. Default namespace has prefix=='',
            // ignore it.
            namespaces.forEach(function(ns) {
                if (ns[0]) {
                    element.setAttributeNS(
                        'http://www.w3.org/2000/xmlns/',
                        'xmlns:' + ns[0], ns[1]);
                }
            });

            currentElement.appendChild(element);
            currentElement = element;
        },

        endElementNS: function(elem, prefix, uri) {
            flushData();
            currentElement = currentElement.parentNode;
        },

        characters: function(chars) {
            currentCharacters += chars;
        },

        cdata: function(cdata) {
            currentCdata += cdata;
        },

        comment: function(comment) {
            currentElement.appendChild(doc.createComment(comment));
        },

        warning: function(msg) {
            // FIXME
        },

        error: function(msg) {
            // FIXME
        }
    };
};
