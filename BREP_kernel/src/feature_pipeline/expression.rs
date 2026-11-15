//! A deterministic recursive-descent evaluator for the practical subset of the
//! app's expression DSL (user directive: move expressions to Rust).
//!
//! The previous app evaluated expressions with a JavaScript `Function()` reading a
//! `configurator` object and a `resolution` default (see
//! `PartHistory.buildExpressionSource`). The REAL expressions are arithmetic, so
//! this ports exactly that arithmetic surface — no arbitrary code eval. Anything outside
//! the supported grammar is a clear `Err`, never a silent wrong answer.
//!
//! Grammar (recursive descent, standard precedence):
//! ```text
//!   source      := statement*
//!   statement   := IDENT '=' expr ';'
//!   expr        := term (('+' | '-') term)*
//!   term        := unary (('*' | '/' | '%') unary)*
//!   unary       := ('-')? primary
//!   primary     := NUMBER
//!                | '(' expr ')'
//!                | dotted ('(' args ')')?
//!   dotted      := IDENT ('.' IDENT)*
//!   args        := (expr (',' expr)*)?
//! ```
//! Supported: f64 literals (incl. scientific notation), identifiers, `+ - * / %`,
//! unary `-`, parentheses, member access `configurator.field`, and a `Math.*`
//! subset: constants `PI`, `E`; unary `sin cos tan sqrt abs floor ceil round`;
//! `pow(a,b)`; variadic `min`/`max`. `//` line comments are skipped.

use std::collections::HashMap;

/// A value in the expression environment: a scalar, or the injected
/// `configurator` object (a flat `field -> f64` map, matching what the
/// `configurator = <values>` prelude exposes to expressions).
#[derive(Debug, Clone)]
enum EvalValue {
    Scalar(f64),
    Object(HashMap<String, f64>),
}

/// The expression environment: the variable bindings produced by evaluating the
/// prelude (`resolution = 32`), the injected `configurator` object, and the
/// user's `expressions` statements in order.
#[derive(Debug, Clone, Default)]
pub struct Env {
    vars: HashMap<String, EvalValue>,
    /// If the env failed to build, every `eval()` returns this error. Numeric
    /// literal params still pass through their own `param_f64` path without ever
    /// calling `eval`, so an expression-free history is unaffected by a poison.
    poison: Option<String>,
}

impl Env {
    /// Build the environment: prelude `resolution = 32`, then the injected
    /// `configurator` object, then the user's `expressions` statements in order
    /// (users may overwrite `resolution`). `configurator_json` may be the full
    /// configurator state (`{ values: {...}, ... }`) or a bare values map — the
    /// `.values` sub-object is used when present, matching the prelude.
    pub fn build(expressions: &str, configurator_json: &serde_json::Value) -> Result<Env, String> {
        let mut env = Env::default();
        env.vars
            .insert("resolution".to_string(), EvalValue::Scalar(32.0));
        env.vars.insert(
            "configurator".to_string(),
            EvalValue::Object(configurator_values(configurator_json)),
        );
        env.run_statements(expressions)?;
        Ok(env)
    }

    /// A poisoned environment: `eval()` reports `msg`; numeric params still pass
    /// through. Lets `execute_history` keep the `-> HistoryResult` signature and
    /// surface an expression-block error per-feature instead of globally.
    pub fn poisoned(msg: String) -> Env {
        Env {
            poison: Some(msg),
            ..Env::default()
        }
    }

    /// Evaluate a single expression string against the environment.
    pub fn eval(&self, source: &str) -> Result<f64, String> {
        if let Some(poison) = &self.poison {
            return Err(format!("expression environment failed to build: {poison}"));
        }
        let tokens = tokenize(source)?;
        let mut parser = Parser::new(&tokens, self);
        let value = parser.parse_expr()?;
        parser.expect_eof()?;
        Ok(value)
    }

    /// Lookup a bound scalar variable (test/introspection helper).
    pub fn get(&self, name: &str) -> Option<f64> {
        match self.vars.get(name) {
            Some(EvalValue::Scalar(value)) => Some(*value),
            _ => None,
        }
    }

    fn run_statements(&mut self, source: &str) -> Result<(), String> {
        let tokens = tokenize(source)?;
        let mut index = 0;
        while index < tokens.len() && tokens[index] != Token::Eof {
            // statement := IDENT '=' expr ';'
            let name = match &tokens[index] {
                Token::Ident(name) => name.clone(),
                other => {
                    return Err(format!(
                        "expected an identifier at the start of a statement, found {other:?}"
                    ))
                }
            };
            index += 1;
            if tokens.get(index) != Some(&Token::Assign) {
                return Err(format!("expected '=' after `{name}` in expression statement"));
            }
            index += 1;
            // Collect the RHS tokens up to the next ';'.
            let start = index;
            while index < tokens.len()
                && tokens[index] != Token::Semi
                && tokens[index] != Token::Eof
            {
                index += 1;
            }
            let rhs = &tokens[start..index];
            let mut parser = Parser::new(rhs, self);
            let value = parser.parse_expr()?;
            parser.expect_eof()?;
            self.vars.insert(name, EvalValue::Scalar(value));
            // Consume the terminating ';' if present.
            if tokens.get(index) == Some(&Token::Semi) {
                index += 1;
            }
        }
        Ok(())
    }
}

/// Put every statement of an expressions source on its own line, in order.
///
/// The only edit is a line break after a statement-ending `;` that has more
/// code after it on the same line: the spaces and tabs after the `;` become a
/// line break plus that line's own indentation. Nothing else moves. A `;`
/// followed (on its line) only by a `//` comment keeps it, so a trailing
/// comment stays with its statement; blank lines, multi-line statements and
/// comment lines are untouched, and the line-break style (`\n` or `\r\n`)
/// follows the source's.
///
/// Conservative by construction:
/// * source that does not tokenize, or whose statements are not
///   `IDENT '=' <well-formed expr> (';' | end)`, is returned exactly as given
///   (no evaluation: an unknown name or a missing configurator field is not a
///   syntax error, and still formats);
/// * the result is returned only when its token stream equals the source's,
///   so it evaluates to exactly the same variables and values;
/// * it is idempotent: formatted text has no `;` with code after it on its
///   line, so formatting it again changes nothing.
pub fn format_expressions(source: &str) -> String {
    let Ok(before) = tokenize(source) else {
        return source.to_string();
    };
    if !statements_well_formed(&before) {
        return source.to_string();
    }
    let bytes = source.as_bytes();
    let newline = if source.contains("\r\n") { "\r\n" } else { "\n" };
    let mut out = String::with_capacity(source.len() + 16);
    let mut copied = 0;
    let mut line_start = 0;
    // The indentation of the line the current statement starts on (a statement
    // wrapped over lines keeps its first line's indent for the ones split
    // after it); `None` until the statement's first code character is seen.
    let mut indent: Option<(usize, usize)> = None;
    let mut statement_started = false;
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'\n' => {
                line_start = i + 1;
                i += 1;
            }
            b'/' if bytes.get(i + 1) == Some(&b'/') => {
                while i < bytes.len() && bytes[i] != b'\n' {
                    i += 1;
                }
            }
            b';' => {
                let mut next = i + 1;
                while next < bytes.len() && (bytes[next] == b' ' || bytes[next] == b'\t') {
                    next += 1;
                }
                let rest_of_line_empty = next == bytes.len()
                    || bytes[next] == b'\n'
                    || bytes[next] == b'\r'
                    || (bytes[next] == b'/' && bytes.get(next + 1) == Some(&b'/'));
                if rest_of_line_empty {
                    statement_started = false;
                } else {
                    let (start, end) = indent.unwrap_or((line_start, line_start));
                    out.push_str(&source[copied..=i]);
                    out.push_str(newline);
                    out.push_str(&source[start..end]);
                    copied = next;
                    // The next statement starts on the line just made, with
                    // the same indent.
                    statement_started = true;
                }
                i = next;
            }
            b' ' | b'\t' | b'\r' => i += 1,
            _ => {
                if !statement_started {
                    let end = source[line_start..]
                        .find(|c: char| c != ' ' && c != '\t')
                        .map_or(source.len(), |offset| line_start + offset);
                    indent = Some((line_start, end));
                    statement_started = true;
                }
                i += 1;
            }
        }
    }
    if copied == 0 {
        return source.to_string();
    }
    out.push_str(&source[copied..]);
    match tokenize(&out) {
        Ok(after) if after == before => out,
        _ => source.to_string(),
    }
}

/// Whether a token stream is a sequence of `IDENT '=' expr` statements, each
/// ended by `;` (the last may end at the end of the source) — the shape
/// [`Env::build`] accepts, checked without evaluating anything.
fn statements_well_formed(tokens: &[Token]) -> bool {
    let mut pos = 0;
    while tokens.get(pos).is_some_and(|token| *token != Token::Eof) {
        if !matches!(tokens.get(pos), Some(Token::Ident(_))) || tokens.get(pos + 1) != Some(&Token::Assign) {
            return false;
        }
        pos += 2;
        if !expr_well_formed(tokens, &mut pos, 0) {
            return false;
        }
        match tokens.get(pos) {
            Some(Token::Semi) => pos += 1,
            Some(Token::Eof) | None => {}
            Some(_) => return false,
        }
    }
    true
}

/// Nesting deeper than this (parentheses or call arguments) is not
/// formatted: the check recurses once per level and must not exhaust the
/// stack on hostile input.
const MAX_FORMAT_DEPTH: usize = 128;

/// `expr` of the module grammar, syntax only; advances `pos` past it.
fn expr_well_formed(tokens: &[Token], pos: &mut usize, depth: usize) -> bool {
    if depth > MAX_FORMAT_DEPTH {
        return false;
    }
    loop {
        // unary := ('-' | '+')* primary
        while matches!(tokens.get(*pos), Some(Token::Minus | Token::Plus)) {
            *pos += 1;
        }
        match tokens.get(*pos) {
            Some(Token::Number(_)) => *pos += 1,
            Some(Token::LParen) => {
                *pos += 1;
                if !expr_well_formed(tokens, pos, depth + 1) || tokens.get(*pos) != Some(&Token::RParen) {
                    return false;
                }
                *pos += 1;
            }
            Some(Token::Ident(_)) => {
                *pos += 1;
                while tokens.get(*pos) == Some(&Token::Dot) {
                    if !matches!(tokens.get(*pos + 1), Some(Token::Ident(_))) {
                        return false;
                    }
                    *pos += 2;
                }
                if tokens.get(*pos) == Some(&Token::LParen) {
                    *pos += 1;
                    if tokens.get(*pos) == Some(&Token::RParen) {
                        *pos += 1;
                    } else {
                        loop {
                            if !expr_well_formed(tokens, pos, depth + 1) {
                                return false;
                            }
                            match tokens.get(*pos) {
                                Some(Token::Comma) => *pos += 1,
                                Some(Token::RParen) => {
                                    *pos += 1;
                                    break;
                                }
                                _ => return false,
                            }
                        }
                    }
                }
            }
            _ => return false,
        }
        // A binary operator continues the expression with another operand.
        match tokens.get(*pos) {
            Some(Token::Plus | Token::Minus | Token::Star | Token::Slash | Token::Percent) => *pos += 1,
            _ => return true,
        }
    }
}

/// Extract the flat `field -> f64` values map from a configurator JSON value,
/// preferring a `.values` sub-object (the normalized configurator state shape).
/// Non-numeric fields are dropped (an expression referencing one errors clearly
/// at member-access time).
fn configurator_values(configurator: &serde_json::Value) -> HashMap<String, f64> {
    let source = configurator
        .get("values")
        .filter(|value| value.is_object())
        .unwrap_or(configurator);
    let mut map = HashMap::new();
    if let Some(object) = source.as_object() {
        for (key, value) in object {
            if let Some(number) = value_as_f64(value) {
                map.insert(key.clone(), number);
            }
        }
    }
    map
}

fn value_as_f64(value: &serde_json::Value) -> Option<f64> {
    match value {
        serde_json::Value::Number(number) => number.as_f64(),
        serde_json::Value::String(text) => text.trim().parse::<f64>().ok(),
        serde_json::Value::Bool(flag) => Some(if *flag { 1.0 } else { 0.0 }),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Tokenizer
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum Token {
    Number(f64),
    Ident(String),
    Plus,
    Minus,
    Star,
    Slash,
    Percent,
    LParen,
    RParen,
    Dot,
    Comma,
    Assign,
    Semi,
    Eof,
}

fn tokenize(source: &str) -> Result<Vec<Token>, String> {
    let bytes = source.as_bytes();
    let mut tokens = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i] as char;
        if c.is_whitespace() {
            i += 1;
            continue;
        }
        // `//` line comment.
        if c == '/' && i + 1 < bytes.len() && bytes[i + 1] as char == '/' {
            while i < bytes.len() && bytes[i] as char != '\n' {
                i += 1;
            }
            continue;
        }
        match c {
            '+' => {
                tokens.push(Token::Plus);
                i += 1;
            }
            '-' => {
                tokens.push(Token::Minus);
                i += 1;
            }
            '*' => {
                tokens.push(Token::Star);
                i += 1;
            }
            '/' => {
                tokens.push(Token::Slash);
                i += 1;
            }
            '%' => {
                tokens.push(Token::Percent);
                i += 1;
            }
            '(' => {
                tokens.push(Token::LParen);
                i += 1;
            }
            ')' => {
                tokens.push(Token::RParen);
                i += 1;
            }
            '.' if !next_is_digit(bytes, i + 1) => {
                tokens.push(Token::Dot);
                i += 1;
            }
            ',' => {
                tokens.push(Token::Comma);
                i += 1;
            }
            '=' => {
                tokens.push(Token::Assign);
                i += 1;
            }
            ';' => {
                tokens.push(Token::Semi);
                i += 1;
            }
            _ if c.is_ascii_digit() || c == '.' => {
                let (number, next) = scan_number(bytes, i)?;
                tokens.push(Token::Number(number));
                i = next;
            }
            _ if c.is_ascii_alphabetic() || c == '_' || c == '$' => {
                let start = i;
                while i < bytes.len() {
                    let ch = bytes[i] as char;
                    if ch.is_ascii_alphanumeric() || ch == '_' || ch == '$' {
                        i += 1;
                    } else {
                        break;
                    }
                }
                tokens.push(Token::Ident(source[start..i].to_string()));
            }
            _ => return Err(format!("unexpected character `{c}` in expression")),
        }
    }
    tokens.push(Token::Eof);
    Ok(tokens)
}

fn next_is_digit(bytes: &[u8], i: usize) -> bool {
    i < bytes.len() && (bytes[i] as char).is_ascii_digit()
}

/// Scan a decimal number with optional fraction and scientific exponent.
fn scan_number(bytes: &[u8], start: usize) -> Result<(f64, usize), String> {
    let mut i = start;
    while next_is_digit(bytes, i) {
        i += 1;
    }
    if i < bytes.len() && bytes[i] as char == '.' {
        i += 1;
        while next_is_digit(bytes, i) {
            i += 1;
        }
    }
    if i < bytes.len() && (bytes[i] as char == 'e' || bytes[i] as char == 'E') {
        let mut j = i + 1;
        if j < bytes.len() && (bytes[j] as char == '+' || bytes[j] as char == '-') {
            j += 1;
        }
        if next_is_digit(bytes, j) {
            i = j;
            while next_is_digit(bytes, i) {
                i += 1;
            }
        }
    }
    let text = std::str::from_utf8(&bytes[start..i]).map_err(|error| error.to_string())?;
    let value = text
        .parse::<f64>()
        .map_err(|_| format!("invalid number literal `{text}`"))?;
    Ok((value, i))
}

// ---------------------------------------------------------------------------
// Parser + evaluator (evaluated directly against a fixed env)
// ---------------------------------------------------------------------------

struct Parser<'a> {
    tokens: &'a [Token],
    pos: usize,
    env: &'a Env,
}

impl<'a> Parser<'a> {
    fn new(tokens: &'a [Token], env: &'a Env) -> Self {
        Self {
            tokens,
            pos: 0,
            env,
        }
    }

    fn peek(&self) -> &Token {
        self.tokens.get(self.pos).unwrap_or(&Token::Eof)
    }

    fn advance(&mut self) -> Token {
        let token = self.tokens.get(self.pos).cloned().unwrap_or(Token::Eof);
        self.pos += 1;
        token
    }

    fn expect_eof(&self) -> Result<(), String> {
        match self.peek() {
            Token::Eof => Ok(()),
            other => Err(format!("unexpected trailing token {other:?} in expression")),
        }
    }

    fn parse_expr(&mut self) -> Result<f64, String> {
        let mut value = self.parse_term()?;
        loop {
            match self.peek() {
                Token::Plus => {
                    self.advance();
                    value += self.parse_term()?;
                }
                Token::Minus => {
                    self.advance();
                    value -= self.parse_term()?;
                }
                _ => break,
            }
        }
        Ok(value)
    }

    fn parse_term(&mut self) -> Result<f64, String> {
        let mut value = self.parse_unary()?;
        loop {
            match self.peek() {
                Token::Star => {
                    self.advance();
                    value *= self.parse_unary()?;
                }
                Token::Slash => {
                    self.advance();
                    value /= self.parse_unary()?;
                }
                Token::Percent => {
                    self.advance();
                    value %= self.parse_unary()?;
                }
                _ => break,
            }
        }
        Ok(value)
    }

    fn parse_unary(&mut self) -> Result<f64, String> {
        if self.peek() == &Token::Minus {
            self.advance();
            return Ok(-self.parse_unary()?);
        }
        if self.peek() == &Token::Plus {
            self.advance();
            return self.parse_unary();
        }
        self.parse_primary()
    }

    fn parse_primary(&mut self) -> Result<f64, String> {
        match self.advance() {
            Token::Number(value) => Ok(value),
            Token::LParen => {
                let value = self.parse_expr()?;
                if self.advance() != Token::RParen {
                    return Err("expected ')'".to_string());
                }
                Ok(value)
            }
            Token::Ident(first) => {
                let mut parts = vec![first];
                while self.peek() == &Token::Dot {
                    self.advance();
                    match self.advance() {
                        Token::Ident(name) => parts.push(name),
                        other => {
                            return Err(format!("expected identifier after '.', found {other:?}"))
                        }
                    }
                }
                if self.peek() == &Token::LParen {
                    self.advance();
                    let args = self.parse_args()?;
                    self.eval_call(&parts.join("."), &args)
                } else {
                    self.eval_reference(&parts)
                }
            }
            other => Err(format!("unexpected token {other:?} in expression")),
        }
    }

    fn parse_args(&mut self) -> Result<Vec<f64>, String> {
        let mut args = Vec::new();
        if self.peek() == &Token::RParen {
            self.advance();
            return Ok(args);
        }
        loop {
            args.push(self.parse_expr()?);
            match self.advance() {
                Token::Comma => continue,
                Token::RParen => break,
                other => return Err(format!("expected ',' or ')' in argument list, found {other:?}")),
            }
        }
        Ok(args)
    }

    fn eval_reference(&self, parts: &[String]) -> Result<f64, String> {
        match parts.len() {
            1 => match self.env.vars.get(&parts[0]) {
                Some(EvalValue::Scalar(value)) => Ok(*value),
                Some(EvalValue::Object(_)) => {
                    Err(format!("`{}` is an object, not a number", parts[0]))
                }
                None => Err(format!("unknown identifier `{}`", parts[0])),
            },
            2 => {
                if parts[0] == "Math" {
                    return match parts[1].as_str() {
                        "PI" => Ok(std::f64::consts::PI),
                        "E" => Ok(std::f64::consts::E),
                        other => Err(format!("unsupported Math constant `Math.{other}`")),
                    };
                }
                match self.env.vars.get(&parts[0]) {
                    Some(EvalValue::Object(map)) => map
                        .get(&parts[1])
                        .copied()
                        .ok_or_else(|| format!("`{}.{}` is not a number", parts[0], parts[1])),
                    Some(EvalValue::Scalar(_)) => {
                        Err(format!("`{}` is a number, not an object", parts[0]))
                    }
                    None => Err(format!("unknown identifier `{}`", parts[0])),
                }
            }
            _ => Err(format!("unsupported member access `{}`", parts.join("."))),
        }
    }

    fn eval_call(&self, name: &str, args: &[f64]) -> Result<f64, String> {
        let expect = |arity: usize| -> Result<(), String> {
            if args.len() == arity {
                Ok(())
            } else {
                Err(format!(
                    "`{name}` expects {arity} argument(s), got {}",
                    args.len()
                ))
            }
        };
        match name {
            "Math.sin" => {
                expect(1)?;
                Ok(args[0].sin())
            }
            "Math.cos" => {
                expect(1)?;
                Ok(args[0].cos())
            }
            "Math.tan" => {
                expect(1)?;
                Ok(args[0].tan())
            }
            "Math.sqrt" => {
                expect(1)?;
                Ok(args[0].sqrt())
            }
            "Math.abs" => {
                expect(1)?;
                Ok(args[0].abs())
            }
            "Math.floor" => {
                expect(1)?;
                Ok(args[0].floor())
            }
            "Math.ceil" => {
                expect(1)?;
                Ok(args[0].ceil())
            }
            "Math.round" => {
                expect(1)?;
                // JavaScript Math.round rounds half up (toward +inf); Rust round() is
                // half-away-from-zero. Match JavaScript.
                Ok((args[0] + 0.5).floor())
            }
            "Math.pow" => {
                expect(2)?;
                Ok(args[0].powf(args[1]))
            }
            "Math.min" => {
                if args.is_empty() {
                    return Ok(f64::INFINITY);
                }
                Ok(args.iter().copied().fold(f64::INFINITY, f64::min))
            }
            "Math.max" => {
                if args.is_empty() {
                    return Ok(f64::NEG_INFINITY);
                }
                Ok(args.iter().copied().fold(f64::NEG_INFINITY, f64::max))
            }
            other => Err(format!("unsupported function `{other}`")),
        }
    }
}

