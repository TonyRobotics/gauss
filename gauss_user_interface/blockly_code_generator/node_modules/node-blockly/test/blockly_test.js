'use strict';

var assert = require('chai').assert;

var Blockly = require('../index.js');

describe('Blockly', function() {
  it('shortestStringLength should return correct length', function() {
    var len = Blockly.utils.shortestStringLength('one,two,three,four,five'.split(','));
    assert.equal(3, len, 'Length of "one"');
    len = Blockly.utils.shortestStringLength('one,two,three,four,five,'.split(','));
    assert.equal(0, len, 'Length of ""');
    len = Blockly.utils.shortestStringLength(['Hello World']);
    assert.equal(11, len, 'List of one');
    len = Blockly.utils.shortestStringLength([]);
    assert.equal(0, len, 'Empty list');
  });
});


