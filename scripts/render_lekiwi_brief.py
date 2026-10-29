#!/usr/bin/env python3
"""Render the HTML brief to PDF using reportlab and beautifulsoup4 (no browser)."""
from html import escape
from pathlib import Path

from bs4 import BeautifulSoup, Tag
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import Image, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/presentations/lekiwi-mujoco-brief.html'
OUTPUT = SOURCE.with_suffix('.pdf')
FONT = 'HeiseiKakuGo-W5'
pdfmetrics.registerFont(UnicodeCIDFont(FONT))
pdfmetrics.registerFontFamily(FONT, normal=FONT, bold=FONT, italic=FONT, boldItalic=FONT)
W, H = landscape(A4)
WIDTH = W - 96
BASE = dict(fontName=FONT, textColor=colors.HexColor('#172a3a'), wordWrap='CJK', alignment=TA_LEFT)
styles = {name: ParagraphStyle(name, fontSize=size, leading=leading, spaceAfter=space, **BASE)
          for name, size, leading, space in [('body', 11.2, 17, 9), ('small', 9.3, 14, 8),
          ('h1', 28, 39, 20), ('h2', 21, 29, 16), ('h3', 14, 21, 8),
          ('kicker', 8, 12, 7), ('big', 27, 36, 8)]}


def inline(node):
    if not isinstance(node, Tag):
        return escape(str(node))
    if node.name == 'br':
        return '<br/>'
    inner = ''.join(inline(c) for c in node.children)
    if node.name in ('strong', 'b'):
        return '<b>' + inner + '</b>'
    if node.name == 'a' and node.get('href', '').startswith('https://'):
        return '<a href="' + escape(node['href'], quote=True) + '" color="#176b87">' + inner + '</a>'
    return inner


def render(node, width=WIDTH):
    if not isinstance(node, Tag) or node.name == 'footer':
        return []
    cls = node.get('class', [])
    if node.name in ('h1', 'h2', 'h3', 'p') or 'kicker' in cls or 'big' in cls or 'pill' in cls:
        key = node.name if node.name in ('h1', 'h2', 'h3') else ('big' if 'big' in cls else 'kicker' if 'kicker' in cls or 'pill' in cls else 'small' if 'small' in cls else 'body')
        return [Paragraph(inline(node), styles[key])]
    if node.name == 'img':
        path = (SOURCE.parent / node['src']).with_suffix('.png')
        image = Image(str(path))
        scale = min(width / image.imageWidth, 285 / image.imageHeight)
        image.drawWidth, image.drawHeight = image.imageWidth * scale, image.imageHeight * scale
        return [image, Spacer(1, 8)]
    if node.name in ('ul', 'ol'):
        return [Paragraph((f'{i}. ' if node.name == 'ol' else '・ ') + inline(li), styles['body'])
                for i, li in enumerate(node.find_all('li', recursive=False), 1)]
    if 'grid' in cls or 'cards' in cls:
        children = node.find_all(recursive=False)
        col = width / len(children)
        cells = [[sum((render(c, col - 25) for c in child.children), []) for child in children]]
        table = Table(cells, colWidths=[col] * len(children))
        table.setStyle(TableStyle([('VALIGN', (0, 0), (-1, -1), 'TOP'),
                                  ('LEFTPADDING', (0, 0), (-1, -1), 10),
                                  ('RIGHTPADDING', (0, 0), (-1, -1), 13),
                                  ('TOPPADDING', (0, 0), (-1, -1), 12),
                                  ('BOTTOMPADDING', (0, 0), (-1, -1), 12),
                                  ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor('#edf2f0'))]))
        return [table, Spacer(1, 14)]
    if node.name == 'table':
        rows = [[Paragraph(inline(cell), styles['body']) for cell in row.find_all(['td', 'th'], recursive=False)]
                for row in node.find_all('tr')]
        table = Table(rows, colWidths=[width * .25, width * .75])
        table.setStyle(TableStyle([('VALIGN', (0, 0), (-1, -1), 'TOP'),
                                  ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#edf2f0')),
                                  ('LINEBELOW', (0, 0), (-1, -1), .4, colors.HexColor('#ccd7db')),
                                  ('TOPPADDING', (0, 0), (-1, -1), 7),
                                  ('BOTTOMPADDING', (0, 0), (-1, -1), 7)]))
        return [table, Spacer(1, 12)]
    if 'note' in cls:
        return [Paragraph(inline(node), styles['small']), Spacer(1, 6)]
    return sum((render(c, width) for c in node.children), [])


def page_background(canvas, doc):
    canvas.saveState()
    canvas.setFillColor(colors.HexColor('#fafbf8'))
    canvas.rect(0, 0, W, H, fill=1, stroke=0)
    canvas.setStrokeColor(colors.HexColor('#ccd7db'))
    canvas.line(48, 30, W - 48, 30)
    canvas.setFillColor(colors.HexColor('#536776'))
    canvas.setFont(FONT, 8)
    canvas.drawString(48, 18, 'RobotNativeEngine  /  LeKiwi + SO-101  /  2026-10-29')
    canvas.drawRightString(W - 48, 18, f'{doc.page:02d} / 06')
    canvas.restoreState()


soup = BeautifulSoup(SOURCE.read_text(), 'html.parser')
slides = soup.select('section.slide')
assert len(slides) == 6
story = []
for i, slide in enumerate(slides):
    if i:
        story.append(PageBreak())
    story.extend(sum((render(c) for c in slide.children), []))
doc = SimpleDocTemplate(str(OUTPUT), pagesize=(W, H), leftMargin=48, rightMargin=48,
                        topMargin=32, bottomMargin=42, title=soup.title.get_text(),
                        author='RobotNativeEngine development', pageCompression=1)
doc.build(story, onFirstPage=page_background, onLaterPages=page_background)
print(OUTPUT)
