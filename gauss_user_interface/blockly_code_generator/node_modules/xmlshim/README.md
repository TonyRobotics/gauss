Node XML Shim
=============

[![Build Status](https://secure.travis-ci.org/znerol/node-xmlshim.svg?branch=2.x)](http://travis-ci.org/znerol/node-xmlshim)

This project brings the ``DOMParser``[1] and ``XMLSerializer``[2] objects known
from the browser to node.js. Additionally functions from
``document.implementation``, most importantly ``createDocument``[3] are exposed
via this module.

XML Shim is ready to be browsified and therefore pretty useful for
cross-platform projects.

Branches:
--------

* *0.x*: Compatible with node versions supported by jsdom 0.11.x
* *1.x*: Compatible with node versions supported by jsdom 3.x (0.10, 0.12)
* *2.x*: Compatible with node versions supported by jsdom >= 3.x (4, 6, 8)

Note that jsdom >= 1.0 does not support CDATA sections. Hence xmlshim >= 1.x
will fail to parse/write any CDATA sections.


DOMParser example:
------------------

    var xmlshim = require('xmlshim');
    var dp = new xmlshim.DOMParser();
    var doc = dp.parseFromString('<hello-world/>', 'text/xml');
    console.log(doc.firstChild.nodeName);


XMLSerializer example:
----------------------

    var xmlshim = require('xmlshim');
    var xs = new xmlshim.XMLSerializer();
    var doc = xmlshim.implementation.createDocument('', '', null);
    var root = doc.createElement('hello-world');
    doc.appendChild(root);
    console.log(xs.serializeToString(doc));


Run the test suite:
-------------------

Run the test suite under node.js:
    
    make test

Run the test suite in firefox web browser (adapt the Makefile or point your
browser manually at browser-test/test.html if you want to test in another
browser):

    make browser-test


Dependencies:
-------------

XML Shim depends on jsdom[4] for the DOM bits as well as a on the
xmlwriter-branch[5] of libxmljs[6] for serialization and parsing. Browserify[7]
and nodeunit is required in order to build and run the browser test suite.


[1]: https://developer.mozilla.org/En/DOMParser
[2]: https://developer.mozilla.org/En/XMLSerializer
[3]: https://developer.mozilla.org/En/DOM/DOMImplementation.createDocument
[4]: https://github.com/tmpvar/jsdom
[5]: https://github.com/znerol/libxmljs/tree/xmlwriter-0.4.2
[6]: https://github.com/polotek/libxmljs
[7]: https://github.com/substack/node-browserify
[8]: https://github.com/caolan/nodeunit
