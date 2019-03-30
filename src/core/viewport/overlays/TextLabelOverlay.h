///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (2018) Alexander Stukowski
//
//  This file is part of OVITO (Open Visualization Tool).
//
//  OVITO is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  OVITO is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once


#include <core/Core.h>
#include <core/dataset/scene/PipelineSceneNode.h>
#include <core/dataset/pipeline/PipelineFlowState.h>
#include <core/utilities/concurrent/SharedFuture.h>
#include <core/rendering/FrameBuffer.h>
#include "ViewportOverlay.h"

namespace Ovito { OVITO_BEGIN_INLINE_NAMESPACE(View) OVITO_BEGIN_INLINE_NAMESPACE(Internal)

/**
 * \brief A viewport overlay that displays a user-defined text label.
 */
class OVITO_CORE_EXPORT TextLabelOverlay : public ViewportOverlay
{
	Q_OBJECT
	OVITO_CLASS(TextLabelOverlay)
	Q_CLASSINFO("DisplayName", "Text label");

public:

	/// Constructor.
	Q_INVOKABLE TextLabelOverlay(DataSet* dataset);

	/// This method asks the overlay to paint its contents over the rendered image.
	virtual void render(const Viewport* viewport, TimePoint time, FrameBuffer* frameBuffer, const ViewProjectionParameters& projParams, const RenderSettings* renderSettings, AsyncOperation& operation) override {
		if(sourceNode()) {
			SharedFuture<PipelineFlowState> stateFuture =  sourceNode()->evaluatePipeline(time);
			if(!operation.waitForFuture(stateFuture))
				return;
			QPainter painter(&frameBuffer->image());
			renderImplementation(painter, renderSettings, stateFuture.result());
		}
		else {
			QPainter painter(&frameBuffer->image());
			renderImplementation(painter, renderSettings, {});
		}
	}

	/// This method asks the overlay to paint its contents over the given interactive viewport.
	virtual void renderInteractive(const Viewport* viewport, TimePoint time, QPainter& painter, const ViewProjectionParameters& projParams, const RenderSettings* renderSettings, AsyncOperation& operation) override {
		const PipelineFlowState& flowState = sourceNode() ? sourceNode()->evaluatePipelinePreliminary(true) : PipelineFlowState();
		renderImplementation(painter, renderSettings, flowState);
	}

	/// Moves the position of the overlay in the viewport by the given amount,
	/// which is specified as a fraction of the viewport render size.
	virtual void moveOverlayInViewport(const Vector2& delta) override {
		setOffsetX(offsetX() + delta.x());
		setOffsetY(offsetY() + delta.y());
	}

public:

	Q_PROPERTY(Ovito::PipelineSceneNode* sourceNode READ sourceNode WRITE setSourceNode);

private:

	/// This method paints the overlay contents onto the given canvas.
	void renderImplementation(QPainter& painter, const RenderSettings* renderSettings, const PipelineFlowState& flowState);

	/// The corner of the viewport where the label is shown in.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(int, alignment, setAlignment, PROPERTY_FIELD_MEMORIZE);

	/// Controls the horizontal offset of label position.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(FloatType, offsetX, setOffsetX, PROPERTY_FIELD_MEMORIZE);

	/// Controls the vertical offset of label position.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(FloatType, offsetY, setOffsetY, PROPERTY_FIELD_MEMORIZE);

	/// Controls the label font.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(QFont, font, setFont, PROPERTY_FIELD_MEMORIZE);

	/// Controls the label font size.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(FloatType, fontSize, setFontSize, PROPERTY_FIELD_MEMORIZE);

	/// The label's text.
	DECLARE_MODIFIABLE_PROPERTY_FIELD(QString, labelText, setLabelText);

	/// The display color of the label.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(Color, textColor, setTextColor, PROPERTY_FIELD_MEMORIZE);

	/// The text outline color.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(Color, outlineColor, setOutlineColor, PROPERTY_FIELD_MEMORIZE);

	/// Controls the outlining of the font.
	DECLARE_MODIFIABLE_PROPERTY_FIELD_FLAGS(bool, outlineEnabled, setOutlineEnabled, PROPERTY_FIELD_MEMORIZE);

	/// The PipelineSceneNode providing global attributes that can be reference in the text.
	DECLARE_MODIFIABLE_REFERENCE_FIELD_FLAGS(PipelineSceneNode, sourceNode, setSourceNode, PROPERTY_FIELD_NO_SUB_ANIM);
};

OVITO_END_INLINE_NAMESPACE
OVITO_END_INLINE_NAMESPACE
}	// End of namespace
