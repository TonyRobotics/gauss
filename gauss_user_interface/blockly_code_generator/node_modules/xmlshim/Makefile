BROWSER=firefox
BROWSERIFY=./node_modules/.bin/browserify
NODEUNIT=./node_modules/.bin/nodeunit

NODEUNIT_SRC=./node_modules/nodeunit
NODEUNIT_DIST=./node_modules/nodeunit/dist
NODEUNIT_JS=$(NODEUNIT_SRC)/dist/browser/nodeunit.js
NODEUNIT_CSS=$(NODEUNIT_SRC)/dist/browser/nodeunit.css

test:
	$(NODEUNIT) test

$(NODEUNIT_DIST):
	(cd $(NODEUNIT_SRC) && npm install && make browser)

browser-test/xmlshim-test.js:
	$(BROWSERIFY) test-browserify-entry.js > browser-test/xmlshim-test.js

browser-test/nodeunit.css: $(NODEUNIT_CSS)
	cp $(NODEUNIT_CSS) browser-test/nodeunit.css

browser-test/nodeunit.js: $(NODEUNIT_JS)
	cp $(NODEUNIT_JS) browser-test/nodeunit.js

browser-test: $(NODEUNIT_DIST) browser-test/xmlshim-test.js browser-test/nodeunit.css browser-test/nodeunit.js
	$(BROWSER) browser-test/test.html >/dev/null 2>&1 &

clean:
	rm -f browser-test/nodeunit.js
	rm -f browser-test/nodeunit.css
	rm -f browser-test/xmlshim-test.js

.PHONY: test browser-test clean
