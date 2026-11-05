import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const mainMjsPath = path.resolve(__dirname, '../node_modules/likec4/__app__/src/main.mjs');

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

if (modified) {
  fs.writeFileSync(mainMjsPath, content, 'utf8');
  console.log('[patch-search] Successfully updated main.mjs');
} else {
  console.log('[patch-search] main.mjs already up to date');
}

