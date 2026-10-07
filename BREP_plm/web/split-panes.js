// Declarative two-pane layouts, including optional controls between the panes.
'use strict';
globalThis.SplitPanes = (() => {
  const instances = new WeakMap();
  let sequence = 0;
  function init(root = document) {
    const layouts = [...(root.matches?.('[data-split-panes]') ? [root] : []), ...root.querySelectorAll('[data-split-panes]')];
    for (const layout of layouts) {
      if (instances.has(layout)) continue;
      const children = [...layout.children];
      if (children.length < 2) continue;
      const left = children[0], right = children.at(-1), middle = children.slice(1, -1);
      left.classList.add('split-pane');
      right.classList.add('split-pane');
      const page = layout.closest('.view');
      const position = page ? [...page.querySelectorAll('[data-split-panes]')].indexOf(layout) : sequence++;
      const id = layout.dataset.splitId || layout.id || `${page?.id || 'page'}-split-${position}`;
      const storageKey = `plm.split-panes:${id}`;
      const minLeft = Number(layout.dataset.splitMinLeft) || 140;
      const minRight = Number(layout.dataset.splitMinRight) || 140;
      const initialLeft = layout.dataset.splitDefaultLeft || '1fr';
      const media = matchMedia(`(min-width: ${Number(layout.dataset.splitBreakpoint) || 721}px)`);
      let share = null, drag = null;
      try {
        const saved = localStorage.getItem(storageKey);
        if (saved !== null && Number(saved) > 0 && Number(saved) < 1) share = Number(saved);
      } catch {}
      left.id ||= `${id}-left`;
      right.id ||= `${id}-right`;
      const divider = document.createElement('div');
      divider.className = 'pane-divider';
      divider.tabIndex = 0;
      divider.setAttribute('role', 'separator');
      divider.setAttribute('aria-orientation', 'vertical');
      divider.setAttribute('aria-label', 'Resize panes');
      divider.setAttribute('aria-controls', `${left.id} ${right.id}`);
      divider.title = 'Drag to resize panes. Arrow keys resize; double-click resets.';
      layout.insertBefore(divider, right);
      layout.classList.add('resizable-panes');
      for (const element of middle) element.classList.add('split-middle');
      function available() {
        const style = getComputedStyle(layout);
        const width = layout.clientWidth - parseFloat(style.paddingLeft) - parseFloat(style.paddingRight);
        const fixed = middle.reduce((sum, element) => {
          const style = getComputedStyle(element);
          return sum + element.getBoundingClientRect().width + parseFloat(style.marginLeft) + parseFloat(style.marginRight);
        }, divider.getBoundingClientRect().width);
        return Math.max(0, width - fixed);
      }
      function bounds() {
        const width = available();
        return {width, min: width ? Math.min(.45, minLeft / width) : .1, max: width ? Math.max(.55, 1 - minRight / width) : .9};
      }
      function updateAria() {
        if (!media.matches || !layout.getClientRects().length) return;
        const {width, min, max} = bounds();
        divider.setAttribute('aria-valuemin', String(Math.round(min * 100)));
        divider.setAttribute('aria-valuemax', String(Math.round(max * 100)));
        divider.setAttribute('aria-valuenow', String(Math.round(width ? left.getBoundingClientRect().width / width * 100 : 50)));
      }
      function apply() {
        layout.classList.toggle('split-panes-active', media.matches);
        divider.hidden = !media.matches;
        const leftTrack = share === null ? initialLeft : `${share}fr`;
        const rightTrack = share === null ? '1fr' : `${1 - share}fr`;
        layout.style.setProperty('--split-columns', `minmax(${minLeft}px, ${leftTrack}) ${middle.map(() => 'auto').join(' ')} 10px minmax(${minRight}px, ${rightTrack})`);
        updateAria();
      }
      function resize(value) {
        const {min, max} = bounds();
        share = Math.max(min, Math.min(max, value));
        apply();
      }
      function remember() {
        try {
          if (share === null) localStorage.removeItem(storageKey);
          else localStorage.setItem(storageKey, String(share));
        } catch {}
      }
      function stop() {
        if (!drag) return;
        const pointerId = drag.pointerId;
        drag = null;
        divider.classList.remove('dragging');
        document.body.classList.remove('resizing-panes');
        if (divider.hasPointerCapture(pointerId)) divider.releasePointerCapture(pointerId);
        remember();
      }
      divider.addEventListener('pointerdown', event => {
        if (event.button !== 0 || !media.matches) return;
        event.preventDefault();
        divider.focus({preventScroll: true});
        drag = {pointerId: event.pointerId, x: event.clientX, width: left.getBoundingClientRect().width};
        divider.setPointerCapture(event.pointerId);
        divider.classList.add('dragging');
        document.body.classList.add('resizing-panes');
      });
      divider.addEventListener('pointermove', event => {
        if (!drag || drag.pointerId !== event.pointerId) return;
        const width = available();
        if (width) resize((drag.width + event.clientX - drag.x) / width);
      });
      for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) divider.addEventListener(name, stop);
      function reset() { share = null; apply(); remember(); }
      divider.addEventListener('dblclick', reset);
      divider.addEventListener('keydown', event => {
        if (!['ArrowLeft', 'ArrowRight', 'Home', 'End', 'Enter'].includes(event.key)) return;
        event.preventDefault();
        if (event.key === 'Enter') { reset(); return; }
        const {width, min, max} = bounds();
        if (!width) return;
        const current = left.getBoundingClientRect().width / width;
        resize(event.key === 'Home' ? min : event.key === 'End' ? max : current + (event.key === 'ArrowLeft' ? -1 : 1) * (event.shiftKey ? 50 : 20) / width);
        remember();
      });
      media.addEventListener('change', () => { stop(); apply(); });
      const observer = new ResizeObserver(updateAria);
      observer.observe(layout);
      instances.set(layout, {reset, observer});
      apply();
    }
  }
  init();
  return {init};
})();
