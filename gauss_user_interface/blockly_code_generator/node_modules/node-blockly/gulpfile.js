'use strict';

var gulp = require('gulp'),
    replace = require('gulp-replace'),
    rename = require("gulp-rename"),
    insert = require('gulp-insert');

var _browserRename = function(path) {
  path.basename += "_browser";
}

var document = `var JSDOM = require('jsdom').JSDOM;
      var window = (new JSDOM()).window;
      var document = window.document;`

gulp.task('blockly', function() {
  return gulp.src('blockly/blockly_compressed.js')
      .pipe(replace(/goog\.global\s*=\s*this;/, 'goog.global=global;'))
      .pipe(insert.wrap(`
      ${document}
      var DOMParser = window.DOMParser;
      var xmlshim = require('xmlshim');
      var XMLSerializer = xmlshim.XMLSerializer;
      var DOMParser = xmlshim.DOMParser; 
      module.exports = (function(){ // `,
          //....ORIGINAL CODE....
          `Blockly.goog=goog;return Blockly;
      })()`))
      .pipe(gulp.dest('lib'))
});

gulp.task('blockly_browser', function() {
  return gulp.src('blockly/blockly_compressed.js')
      .pipe(replace(/goog\.global\s*=\s*this;/, 'goog.global=window;'))
      .pipe(insert.wrap(`
      module.exports = (function(){ //`,
          //....ORIGINAL CODE....
          `Blockly.goog=goog;return Blockly;
      })()`))
      .pipe(rename(_browserRename))
      .pipe(gulp.dest('lib'))
});

gulp.task('blocks', function() {
  return gulp.src('blockly/blocks_compressed.js')
      .pipe(insert.wrap(`
        module.exports = function(Blockly){
          var goog = Blockly.goog;
          ${document}
          Blockly.Blocks={};`,
          //....ORIGINAL CODE....
          `return Blockly.Blocks;
        }`))
      .pipe(gulp.dest('lib'))
});

gulp.task('blocks_browser', function() {
  return gulp.src('blockly/blocks_compressed.js')
      .pipe(insert.wrap(`
        module.exports = function(Blockly){
          var goog = Blockly.goog;
          Blockly.Blocks={};`,
          //....ORIGINAL CODE....
          `return Blockly.Blocks;
        }`))
      .pipe(rename(_browserRename))
      .pipe(gulp.dest('lib'))
});

gulp.task('js', function() {
  return gulp.src('blockly/javascript_compressed.js')
      .pipe(insert.wrap('module.exports = function(Blockly){', 'return Blockly.JavaScript;}'))
      .pipe(gulp.dest('lib'))
});

gulp.task('dart', function() {
  return gulp.src('blockly/dart_compressed.js')
      .pipe(insert.wrap('module.exports = function(Blockly){', 'return Blockly.Dart;}'))
      .pipe(replace(/window\./g, ''))
      .pipe(gulp.dest('lib'))
});

gulp.task('python', function() {
  return gulp.src('blockly/python_compressed.js')
      .pipe(insert.wrap('module.exports = function(Blockly){', 'return Blockly.Python;}'))
      .pipe(gulp.dest('lib'))
});

gulp.task('php', function() {
  return gulp.src('blockly/php_compressed.js')
      .pipe(insert.wrap('module.exports = function(Blockly){', 'return Blockly.PHP;}'))
      .pipe(gulp.dest('lib'))
});

gulp.task('lua', function() {
  return gulp.src('blockly/lua_compressed.js')
      .pipe(insert.wrap('module.exports = function(Blockly){', 'return Blockly.Lua;}'))
      .pipe(gulp.dest('lib'))
});

gulp.task('i18n', function() {
  return gulp.src('blockly/msg/js/*.js')
    .pipe(replace(/goog\.[^\n]+/g, ''))
    .pipe(insert.wrap('var Blockly = {}; Blockly.Msg={};  module.exports = function(){ ', 'return Blockly.Msg;}'))
    .pipe(gulp.dest('lib/i18n/'))
});

gulp.task('build', [
  'blocks',
  'blocks_browser',
  'blockly',
  'blockly_browser',
  'i18n',
  'js',
  'php',
  'dart',
  'python',
  'lua',
]);




