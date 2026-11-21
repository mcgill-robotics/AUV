import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const mainMjsPath = path.resolve(__dirname, '../node_modules/likec4/__app__/src/main.mjs');
const styleCssPath = path.resolve(__dirname, '../node_modules/likec4/__app__/src/style.css');

if (!fs.existsSync(mainMjsPath)) {
  console.log('[patch-search] main.mjs not found, skipping');
  process.exit(0);
}

let content = fs.readFileSync(mainMjsPath, 'utf8');
let modified = false;

// 1. Patch useHandleElementSelection to navigate directly to defaultView / scopedView (schematic)
const handleTarget = `function useHandleElementSelection() {
	const $ = c(2);
	const ctx = useSearchContext();
	let t0;
	if ($[0] !== ctx) {
		t0 = (element) => {
			const views = [...element.views()];
			if (views.length === 0) return;
			const elementFqn = element.id;
			const onlyOneViewId = t$10(views)?.id;
			if (!onlyOneViewId) {
				ctx.openPickView(elementFqn);
				return;
			}
			ctx.navigateTo(onlyOneViewId, elementFqn);
		};`;

const handleReplacement = `function useHandleElementSelection() {
	const $ = c(2);
	const ctx = useSearchContext();
	let t0;
	if ($[0] !== ctx) {
		t0 = (element) => {
			const defaultView = element.defaultView ?? [...element.scopedViews()][0];
			if (defaultView) {
				ctx.navigateTo(defaultView.id);
				return;
			}
			const views = [...element.views()];
			if (views.length === 0) return;
			const elementFqn = element.id;
			const onlyOneViewId = t$10(views)?.id;
			if (!onlyOneViewId) {
				ctx.openPickView(elementFqn);
				return;
			}
			ctx.navigateTo(onlyOneViewId, elementFqn);
		};`;

if (content.includes(handleTarget)) {
  content = content.replace(handleTarget, handleReplacement);
  modified = true;
  console.log('[patch-search] Patched useHandleElementSelection to link directly to scoped schematic view');
}

// 2. Patch PickView to properly include scoped views
const pickTarget = `		const element = t1.element(elementFqn);
		scoped = [];
		others = [];
		for (const view of element.views()) if (view.viewOf === element) scoped.push(view);
		else others.push(view);`;

const pickReplacement = `		const element = t1.element(elementFqn);
		scoped = [...element.scopedViews()];
		others = [...element.views()].filter(v => v.viewOf !== element);`;

if (content.includes(pickTarget)) {
  content = content.replace(pickTarget, pickReplacement);
  modified = true;
  console.log('[patch-search] Patched PickView to include scopedViews');
}

// 3. Patch search matching for multi-word / natural queries
const elemSearchTarget = `				t2 = (t3) => {
					const { element: element_4 } = t3;
					return (element_4.title + " " + element_4.name + " " + (element_4.summary.md || "")).toLocaleLowerCase().includes(search);
				};`;

const elemSearchReplacement = `				t2 = (t3) => {
					const { element: element_4 } = t3;
					const words = search.toLowerCase().replace(/_/g, ' ').split(/\\s+/).filter(Boolean);
					const text = (element_4.title + " " + element_4.name + " " + (element_4.summary.md || "") + " " + element_4.id).toLowerCase().replace(/_/g, ' ');
					return words.every(w => text.includes(w));
				};`;

if (content.includes(elemSearchTarget)) {
  content = content.replace(elemSearchTarget, elemSearchReplacement);
  modified = true;
  console.log('[patch-search] Patched element search to support flexible multi-word matching');
}

const viewSearchTarget = `					const desc = preferSummary(view.$view)?.md || " ";
					return \`\${view.id} \${view.title} \${desc}\`.toLocaleLowerCase().includes(search);`;

const viewSearchReplacement = `					const desc = preferSummary(view.$view)?.md || " ";
					const words = search.toLowerCase().replace(/_/g, ' ').split(/\\s+/).filter(Boolean);
					const text = \`\${view.id} \${view.title} \${desc}\`.toLowerCase().replace(/_/g, ' ');
					return words.every(w => text.includes(w));`;

if (content.includes(viewSearchTarget)) {
  content = content.replace(viewSearchTarget, viewSearchReplacement);
  modified = true;
  console.log('[patch-search] Patched view search to support flexible multi-word matching');
}

// 4. Patch main.mjs with SVG Zoom Lightbox script
const lightboxMarker = '/* [patch-svg-lightbox] */';
if (!content.includes(lightboxMarker)) {
  const lightboxScript = `
${lightboxMarker}
if (typeof window !== 'undefined') {
  (function initSvgZoomLightbox() {
    function setup() {
      if (document.getElementById('likec4-svg-lightbox')) return;

      const lightbox = document.createElement('div');
      lightbox.id = 'likec4-svg-lightbox';
      lightbox.innerHTML = \`
        <div id="likec4-svg-lightbox-header">
          <div id="likec4-svg-lightbox-title">SVG Diagram</div>
          <div id="likec4-svg-lightbox-tools">
            <button type="button" class="likec4-lightbox-btn" id="likec4-zoom-out" title="Zoom Out (-)">−</button>
            <span id="likec4-zoom-level">100%</span>
            <button type="button" class="likec4-lightbox-btn" id="likec4-zoom-in" title="Zoom In (+)">+</button>
            <button type="button" class="likec4-lightbox-btn" id="likec4-zoom-reset" title="Reset Zoom (0)">Fit</button>
            <a id="likec4-zoom-raw" class="likec4-lightbox-btn" target="_blank" rel="noopener noreferrer" title="Open original SVG in new tab">↗ Open Raw</a>
            <button type="button" class="likec4-lightbox-btn primary" id="likec4-zoom-close" title="Close (Esc)">✕</button>
          </div>
        </div>
        <div id="likec4-svg-lightbox-viewport">
          <img id="likec4-svg-lightbox-img" src="" alt="Zoomed SVG" draggable="false" />
          <div id="likec4-svg-lightbox-footer">
            <span> Wheel to Zoom</span>
            <span> Drag to Pan</span>
            <span> Double-click for 2.5x</span>
            <span>[Esc] to Close</span>
          </div>
        </div>
      \`;
      document.body.appendChild(lightbox);

      const img = lightbox.querySelector('#likec4-svg-lightbox-img');
      const title = lightbox.querySelector('#likec4-svg-lightbox-title');
      const zoomLevel = lightbox.querySelector('#likec4-zoom-level');
      const rawLink = lightbox.querySelector('#likec4-zoom-raw');
      const viewport = lightbox.querySelector('#likec4-svg-lightbox-viewport');
      const btnIn = lightbox.querySelector('#likec4-zoom-in');
      const btnOut = lightbox.querySelector('#likec4-zoom-out');
      const btnReset = lightbox.querySelector('#likec4-zoom-reset');
      const btnClose = lightbox.querySelector('#likec4-zoom-close');

      let scale = 1;
      let posX = 0;
      let posY = 0;
      let isDragging = false;
      let startX = 0;
      let startY = 0;
      let dragMoved = false;

      function updateTransform() {
        img.style.transform = \`translate(\${posX}px, \${posY}px) scale(\${scale})\`;
        zoomLevel.textContent = \`\${Math.round(scale * 100)}%\`;
      }

      function setScale(newScale, clientX, clientY) {
        const clampedScale = Math.min(Math.max(newScale, 0.25), 12);
        if (clientX !== undefined && clientY !== undefined) {
          const rect = viewport.getBoundingClientRect();
          const originX = clientX - (rect.left + rect.width / 2) - posX;
          const originY = clientY - (rect.top + rect.height / 2) - posY;
          const ratio = clampedScale / scale - 1;
          posX -= originX * ratio;
          posY -= originY * ratio;
        }
        scale = clampedScale;
        updateTransform();
      }

      function resetView() {
        scale = 1;
        posX = 0;
        posY = 0;
        updateTransform();
      }

      function openLightbox(src, altText) {
        img.src = src;
        rawLink.href = src;
        const name = altText || src.split('/').pop().replace(/\\?.*$/, '').replace(/\\.svg$/i, '') || 'SVG Drawing';
        title.textContent = name;
        resetView();
        lightbox.classList.add('active');
      }

      function closeLightbox() {
        lightbox.classList.remove('active');
      }

      btnIn.addEventListener('click', (e) => { e.stopPropagation(); setScale(scale * 1.3); });
      btnOut.addEventListener('click', (e) => { e.stopPropagation(); setScale(scale / 1.3); });
      btnReset.addEventListener('click', (e) => { e.stopPropagation(); resetView(); });
      btnClose.addEventListener('click', (e) => { e.stopPropagation(); closeLightbox(); });

      viewport.addEventListener('wheel', (e) => {
        e.preventDefault();
        const factor = e.deltaY < 0 ? 1.15 : (1 / 1.15);
        setScale(scale * factor, e.clientX, e.clientY);
      }, { passive: false });

      viewport.addEventListener('mousedown', (e) => {
        if (e.button !== 0) return;
        isDragging = true;
        dragMoved = false;
        startX = e.clientX - posX;
        startY = e.clientY - posY;
        viewport.classList.add('dragging');
      });

      window.addEventListener('mousemove', (e) => {
        if (!isDragging) return;
        const nextX = e.clientX - startX;
        const nextY = e.clientY - startY;
        if (Math.abs(nextX - posX) > 3 || Math.abs(nextY - posY) > 3) {
          dragMoved = true;
        }
        posX = nextX;
        posY = nextY;
        updateTransform();
      });

      window.addEventListener('mouseup', () => {
        if (isDragging) {
          isDragging = false;
          viewport.classList.remove('dragging');
        }
      });

      viewport.addEventListener('click', (e) => {
        if (dragMoved) return;
        if (e.target === viewport) {
          closeLightbox();
        }
      });

      viewport.addEventListener('dblclick', (e) => {
        if (e.target === viewport) return;
        if (scale <= 1.25) {
          setScale(2.5, e.clientX, e.clientY);
        } else {
          resetView();
        }
      });

      window.addEventListener('keydown', (e) => {
        if (!lightbox.classList.contains('active')) return;
        if (e.key === 'Escape') {
          closeLightbox();
        } else if (e.key === '+' || e.key === '=') {
          setScale(scale * 1.25);
        } else if (e.key === '-' || e.key === '_') {
          setScale(scale / 1.25);
        } else if (e.key === '0' || e.key.toLowerCase() === 'r') {
          resetView();
        }
      });

      // Global capture click listener for all diagram images and SVGs
      document.addEventListener('click', (e) => {
        const target = e.target;
        if (!(target instanceof HTMLElement)) return;
        const imgEl = target.closest('img');
        if (!imgEl || !imgEl.src) return;

        const src = imgEl.src.toLowerCase();
        const isSvg = src.includes('.svg') || src.includes('/img/');
        const isInsideDiagram = !!imgEl.closest('[data-likec4-node-description], .likec4-markdown-block, .react-flow__node');

        if (isSvg || isInsideDiagram) {
          e.preventDefault();
          e.stopPropagation();
          openLightbox(imgEl.src, imgEl.alt || imgEl.getAttribute('title') || '');
        }
      }, true);
    }

    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', setup);
    } else {
      setup();
    }
  })();
}
`;
  content += lightboxScript;
  modified = true;
  console.log('[patch-search] Appended SVG Zoom Lightbox script to main.mjs');
}

if (modified) {
  fs.writeFileSync(mainMjsPath, content, 'utf8');
  console.log('[patch-search] Successfully updated main.mjs');
} else {
  console.log('[patch-search] main.mjs already up to date');
}

// 5. Ensure style.css has custom image, view highlighting, and lightbox styling
if (fs.existsSync(styleCssPath)) {
  let cssContent = fs.readFileSync(styleCssPath, 'utf8');
  const cssMarker = '/* [patch-style] custom image & view highlighting & svg zoom */';
  const legacyMarker = '/* [patch-style] custom image & view highlighting */';

  const customCss = `
${cssMarker}
/* 1. Prevent descriptions and images from disappearing at low/fit zoom */
:where([data-likec4-zoom-small="true"]) .likec4-element-node-data [data-likec4-node-description] {
  display: -webkit-box !important;
}
:where([data-likec4-zoom-small="true"]) .action-buttons {
  display: flex !important;
}

/* 2. Fit drawings/images cleanly inside element boxes */
.likec4-element-node-data [data-likec4-node-description] {
  max-height: none !important;
  -webkit-line-clamp: unset !important;
  overflow: visible !important;
}
.likec4-element-node-data [data-likec4-node-description] img {
  max-width: 100% !important;
  max-height: 120px !important;
  width: auto !important;
  height: auto !important;
  object-fit: contain !important;
  margin: 4px auto !important;
  display: block !important;
  border-radius: 4px;
}

/* 3. Highlight any box that contains a drill-down view */
:where(.react-flow__node):has([aria-label="Navigate to view"]) .likec4-element-shape,
:where(.react-flow__node):has([aria-label="Navigate to view"]) .likec4-compound-container {
  box-shadow: 0 0 0 2.5px #1971c2, 0 4px 14px rgba(25, 113, 194, 0.35) !important;
  cursor: pointer !important;
}
:where(.react-flow__node):has([aria-label="Navigate to view"]):hover .likec4-element-shape,
:where(.react-flow__node):has([aria-label="Navigate to view"]):hover .likec4-compound-container {
  box-shadow: 0 0 0 3.5px #1864ab, 0 6px 20px rgba(24, 100, 171, 0.5) !important;
}

/* 4. Always show the drill-down navigate action button on view-containing boxes */
:where(.react-flow__node):has([aria-label="Navigate to view"]) .action-buttons {
  display: flex !important;
  opacity: 0.95 !important;
  visibility: visible !important;
}
:where(.react-flow__node) .action-btn[aria-label="Navigate to view"] {
  background-color: #1971c2 !important;
  color: #ffffff !important;
  border-radius: 4px !important;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.3) !important;
}

/* 5. SVG and image zoom cursor & subtle hover feedback */
.likec4-element-node-data [data-likec4-node-description] img,
.likec4-markdown-block img,
img[src*="/img/"],
img[src$=".svg"] {
  cursor: zoom-in !important;
  transition: transform 0.15s ease, filter 0.15s ease;
}
.likec4-element-node-data [data-likec4-node-description] img:hover,
.likec4-markdown-block img:hover {
  filter: drop-shadow(0 2px 8px rgba(25, 113, 194, 0.4));
}

/* 6. SVG Lightbox Overlay */
#likec4-svg-lightbox {
  position: fixed;
  inset: 0;
  z-index: 999999;
  background: rgba(10, 15, 26, 0.88);
  backdrop-filter: blur(10px);
  -webkit-backdrop-filter: blur(10px);
  display: flex;
  flex-direction: column;
  opacity: 0;
  pointer-events: none;
  transition: opacity 0.2s ease;
  user-select: none;
}
#likec4-svg-lightbox.active {
  opacity: 1;
  pointer-events: auto;
}
#likec4-svg-lightbox-header {
  height: 52px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 16px;
  background: rgba(15, 23, 42, 0.7);
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
  color: #f1f5f9;
  font-family: var(--mantine-font-family, system-ui, sans-serif);
  font-size: 14px;
}
#likec4-svg-lightbox-title {
  font-weight: 600;
  color: #e2e8f0;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  max-width: 50vw;
}
#likec4-svg-lightbox-tools {
  display: flex;
  align-items: center;
  gap: 8px;
}
#likec4-zoom-level {
  min-width: 48px;
  text-align: center;
  font-variant-numeric: tabular-nums;
  font-size: 13px;
  color: #94a3b8;
  font-weight: 600;
}
.likec4-lightbox-btn {
  background: rgba(255, 255, 255, 0.08);
  border: 1px solid rgba(255, 255, 255, 0.15);
  color: #e2e8f0;
  padding: 6px 12px;
  border-radius: 6px;
  font-size: 13px;
  font-weight: 500;
  cursor: pointer;
  display: inline-flex;
  align-items: center;
  text-decoration: none;
  gap: 6px;
  transition: background 0.15s, border-color 0.15s;
}
.likec4-lightbox-btn:hover {
  background: rgba(255, 255, 255, 0.18);
  border-color: rgba(255, 255, 255, 0.3);
  color: #ffffff;
}
.likec4-lightbox-btn.primary {
  background: #1971c2;
  border-color: #228be6;
  color: #ffffff;
}
.likec4-lightbox-btn.primary:hover {
  background: #1864ab;
}
#likec4-svg-lightbox-viewport {
  flex: 1;
  position: relative;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: grab;
}
#likec4-svg-lightbox-viewport.dragging {
  cursor: grabbing;
}
#likec4-svg-lightbox-img {
  max-width: 90vw;
  max-height: 85vh;
  object-fit: contain;
  transform-origin: center center;
  box-shadow: 0 10px 40px rgba(0, 0, 0, 0.5);
  border-radius: 6px;
  background: rgba(255, 255, 255, 0.02);
}
#likec4-svg-lightbox-footer {
  position: absolute;
  bottom: 16px;
  left: 50%;
  transform: translateX(-50%);
  background: rgba(15, 23, 42, 0.85);
  backdrop-filter: blur(8px);
  -webkit-backdrop-filter: blur(8px);
  border: 1px solid rgba(255, 255, 255, 0.15);
  padding: 6px 16px;
  border-radius: 20px;
  font-size: 12px;
  color: #94a3b8;
  display: flex;
  align-items: center;
  gap: 16px;
  box-shadow: 0 4px 16px rgba(0, 0, 0, 0.4);
  pointer-events: none;
}
`;

  if (cssContent.includes(cssMarker)) {
    console.log('[patch-search] style.css already has current custom styles');
  } else if (cssContent.includes(legacyMarker)) {
    // Replace legacy patch
    const idx = cssContent.indexOf(legacyMarker);
    cssContent = cssContent.slice(0, idx) + customCss;
    fs.writeFileSync(styleCssPath, cssContent, 'utf8');
    console.log('[patch-search] Updated legacy custom styles in style.css');
  } else {
    cssContent += customCss;
    fs.writeFileSync(styleCssPath, cssContent, 'utf8');
    console.log('[patch-search] Appended custom styles to style.css');
  }
}


