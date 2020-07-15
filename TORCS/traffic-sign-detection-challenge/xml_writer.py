# for python 2
# import HTMLParser
# for python 3
import html.parser
from xml.dom import getDOMImplementation


def _create_element_with_text(document, tag, text):
    elem = document.createElement(tag)
    text_node = document.createTextNode(text)
    elem.appendChild(text_node)
    return elem


def write_xml(dets, path):
    impl = getDOMImplementation()
    doc = impl.createDocument(None, 'opencv_storage', None)
    root = doc.documentElement
    # trans = HTMLParser.HTMLParser()
    for i, frame_det in enumerate(dets):
        num_tar = 0
        for j in range(len(frame_det)):
            tp = frame_det[j]['Type']
            if tp == 'others':
                print('\tskip the others label')
            else:
                num_tar += 1
        tag = 'Frame%0.5dTargetNumber' % i
        node = _create_element_with_text(doc, tag, str(num_tar))
        root.appendChild(node)
        true_cnter = 0
        for j in range(len(frame_det)):
            pos = frame_det[j]['Position']
            tp = frame_det[j]['Type']
            if tp == 'others':
                # print('\tskip the others label')
                continue
            tag = 'Frame%0.5dTarget%0.5d' % (i, true_cnter)
            node = doc.createElement(tag)
            type_node = _create_element_with_text(doc, 'Type', tp)
            pos_node = _create_element_with_text(doc, 'Position',
                                                 str(pos[0]) + ' ' +
                                                 str(pos[1]) + ' ' +
                                                 str(pos[2]) + ' ' +
                                                 str(pos[3]))
            node.appendChild(type_node)
            node.appendChild(pos_node)
            root.appendChild(node)
            true_cnter += 1
    context = doc.toprettyxml(encoding='UTF-8')
    # For python 2
    # context = trans.unescape(context.decode('gbk'))
    # For python 3
    # context = html.parser.unescape(context.decode('gbk'))
    # context = context.replace('gbk', 'zh_CN.UTF-8')
    with open(path, 'w') as f:
        f.writelines(context.decode('UTF-8'))