function DOMWriter(writer) {
    this.writer = writer;
}

DOMWriter.prototype.writeNode = function(node) {
    switch (node.nodeType) {
        case node.ELEMENT_NODE:
            this.writeElement(node);
            break;

        case node.ATTRIBUTE_NODE:
            this.writeAttribute(node);
            break;

        case node.TEXT_NODE:
            this.writeText(node);
            break;

        case node.CDATA_SECTION_NODE:
            this.writeCdata(node);
            break;

        case node.COMMENT_NODE:
            this.writeComment(node);
            break;

        case node.DOCUMENT_NODE:
            this.writeDocument(node);
            break;

        case node.DOCUMENT_FRAGMENT_NODE:
            this.writeDocumentFragment(node);
            break;

        default:
            console.log('Serialization of node type ' + node.nodeType +
                    ' not supported yet');
            break;
    }
};


DOMWriter.prototype.writeElement = function(element) {
    var i, prefix, nsURI, name, attr;

    name = element.nodeName.split(':').slice(-1)[0];
    nsURI = element.namespaceURI || undefined;
    prefix = element.prefix || undefined;

    this.writer.startElementNS(prefix, name, nsURI);

    for (i=0; i < element.attributes.length; i++) {
        attr = element.attributes[i];
        if (attr.name.substr(0, 6) !== 'xmlns:') {
            this.writeAttribute(element.attributes[i]);
        }
    }

    for (i=0; i < element.childNodes.length; i++) {
        this.writeNode(element.childNodes[i]);
    }

    this.writer.endElement();
};

DOMWriter.prototype.writeAttribute = function(attribute) {
    var name, nsURI, prefix;

    name = attribute.name.split(':').slice(-1)[0];
    nsURI = attribute.namespaceURI || undefined;
    prefix = attribute.prefix || undefined;

    this.writer.startAttributeNS(prefix, name, nsURI);
    this.writer.writeString(attribute.value);
    this.writer.endAttribute();
};

DOMWriter.prototype.writeText = function(text) {
    this.writer.writeString(text.data);
};

DOMWriter.prototype.writeCdata = function(cdata) {
    this.writer.startCdata();
    this.writer.writeString(cdata.data);
    this.writer.endCdata();
};

DOMWriter.prototype.writeComment = function(comment) {
    this.writer.startComment();
    this.writer.writeString(comment.data);
    this.writer.endComment();
}

DOMWriter.prototype.writeDocument = function(doc) {
    this.writeElement(doc.documentElement);
}

DOMWriter.prototype.writeDocumentFragment = function(docfrag) {
    var i;
    for (i=0; i < docfrag.childNodes.length; i++) {
        this.writeNode(docfrag.childNodes[i]);
    }
}

exports.DOMWriter = DOMWriter;
