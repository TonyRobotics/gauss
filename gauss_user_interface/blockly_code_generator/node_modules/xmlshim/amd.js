/**
 * AMD version of xmlshim. Copy this file into your AMD modules directory and
 * rename it to xmlshim.js
 */
define(function() {
    return {
        'XMLSerializer': (typeof(XMLSerializer) !== 'undefined') ? XMLSerializer : undefined,
        'DOMParser': (typeof(DOMParser) !== 'undefined') ? DOMParser : undefined,
        'implementation': (typeof(document) !== 'undefined') ? document.implementation : undefined
    };
});
