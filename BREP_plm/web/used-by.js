// The question asked before a revision is made obsolete or a draft deleted:
// it names the assemblies that use the revision, so whoever answers knows
// what they are retiring it from (owner rule: prompts, not refusals). The
// answer defaults to proceed; a where-used that cannot be read asks the plain
// question.
(() => {
  const enc = encodeURIComponent;
  // How many users the question names before "and N more" (the server's
  // warnings and refusals name as many).
  const NAMED_AT_MOST = 10;

  function named(lines) {
    const names = [];
    for (const line of lines || []) {
      if (line.level !== 1) continue;
      const name = `${line.number} rev ${line.revision_label} (${line.state})`;
      if (!names.includes(name)) names.push(name);
    }
    const shown = names.slice(0, NAMED_AT_MOST).join(', ');
    const more = names.length - NAMED_AT_MOST;
    return { count: names.length, text: more > 0 ? `${shown} and ${more} more` : shown };
  }

  // `action` is 'obsolete' or 'delete'; `used` a where-used answer for the
  // revision (null when it could not be read).
  function question(action, label, used) {
    const users = named(used?.lines);
    if (action === 'obsolete') {
      const ask = `Mark rev ${label} obsolete?`;
      if (!used) return ask;
      if (!users.count) return `${ask}\n\nNothing uses it.`;
      return `${ask}\n\nIt is still used by ${users.text}. They keep using it; their BOMs flag it until they are revised to use another revision.`;
    }
    const ask = `Delete draft rev ${label} and its document?`;
    if (!used || !users.count) return ask;
    return `${ask}\n\nIt is used by ${users.text}. A list that names this revision refuses the delete; a floating use moves to another revision.`;
  }

  async function ask(api, part, revision, label, action) {
    let used = null;
    try {
      used = await api('GET', `/api/parts/${enc(part)}/where-used?revision=${enc(revision)}&levels=1`);
    } catch (_) {
      used = null;
    }
    return question(action, label, used);
  }

  globalThis.UsedBy = { question, ask, named };
})();
