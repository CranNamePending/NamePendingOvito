////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2013 Alexander Stukowski
//
//  This file is part of OVITO (Open Visualization Tool).
//
//  OVITO is free software; you can redistribute it and/or modify it either under the
//  terms of the GNU General Public License version 3 as published by the Free Software
//  Foundation (the "GPL") or, at your option, under the terms of the MIT License.
//  If you do not alter this notice, a recipient may use your version of this
//  file under either the GPL or the MIT License.
//
//  You should have received a copy of the GPL along with this program in a
//  file LICENSE.GPL.txt.  You should have received a copy of the MIT License along
//  with this program in a file LICENSE.MIT.txt
//
//  This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND,
//  either express or implied. See the GPL or the MIT License for the specific language
//  governing rights and limitations.
//
////////////////////////////////////////////////////////////////////////////////////////

#include <ovito/gui/desktop/GUI.h>
#include "ElidedTextLabel.h"

namespace Ovito {

/******************************************************************************
* Returns the rect that is available for us to draw the document
******************************************************************************/
QRect ElidedTextLabel::documentRect() const
{
	QRect cr = contentsRect();
	int m = margin();
	cr.adjust(m, m, -m, -m);
	m = indent();
    if(m < 0 && frameWidth()) // no indent, but we do have a frame
    	m = fontMetrics().width(QLatin1Char('x')) / 2 - margin();
    int align = QStyle::visualAlignment(layoutDirection(), alignment());
    if(m > 0) {
		if (align & Qt::AlignLeft)
            cr.setLeft(cr.left() + m);
        if (align & Qt::AlignRight)
            cr.setRight(cr.right() - m);
        if (align & Qt::AlignTop)
            cr.setTop(cr.top() + m);
        if (align & Qt::AlignBottom)
            cr.setBottom(cr.bottom() - m);
    }
    return cr;
}

/******************************************************************************
* Paints the widget.
******************************************************************************/
void ElidedTextLabel::paintEvent(QPaintEvent *)
{
    QStyle *style = QWidget::style();
    QPainter painter(this);
    QRect cr = documentRect();
    int flags = QStyle::visualAlignment(layoutDirection(), alignment());
    QString elidedText = painter.fontMetrics().elidedText(text(), Qt::ElideLeft, cr.width(), flags);
    style->drawItemText(&painter, cr, flags, palette(), isEnabled(), elidedText, foregroundRole());

    // Use the label's full text as tool tip.
    if(toolTip() != text())
    	setToolTip(text());
}

}	// End of namespace
